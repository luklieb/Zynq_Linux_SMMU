#include <linux/init.h>           
#include <linux/module.h>         
#include <linux/device.h>         
#include <linux/kernel.h>         
#include <linux/fs.h>            
#include <linux/uaccess.h>         
#include <linux/sched.h>
#include <linux/iommu.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/list.h>
#include <linux/syscalls.h>
#include <linux/mutex.h>
#include <linux/idr.h>
#include <asm/cputype.h>
#include <asm/cpufeature.h>
#include <linux/spinlock.h>
#include <linux/time.h>

#include "mod.h"
#include <linux/hsa-smmu.h>


#define SMMU_DEBUG

#undef PDEBUG             /* undef it, just in case */
#ifdef SMMU_DEBUG
#define PDEBUG(fmt, args...)            printk( KERN_INFO "hsachar: " fmt, ## args)
#else
#define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */

#define CLASS_NAME      "hsa"
#define MISC_NAME       "hsa"
#define DEVICE_NAME	"smmu"
#define MISC_MINOR      0
#define DEVICE_MINOR    1
#define PROC_SHIFT	8

#define SID_BASE	(u16) 0x200
//inclusive
#define AXIID_START	(u8) 0
//exclusive
#define AXIID_END	(u8) 16

#define NANO	1000000000

static int		majorNumber;                  
static int		numberOpens = 0;             
static struct		class *class  = NULL; 
static struct		device *device = NULL; 
static struct		device *smmu_device = NULL;

static int		dev_open(struct inode *, struct file *);
static int		dev_release(struct inode *, struct file *);
static ssize_t		dev_read(struct file *, char *, size_t, loff_t *);
static int		dev_mmap(struct file *, struct vm_area_struct *);
static char		*hsa_devnode(struct device *, umode_t *);
static int		hsa_bind(void);
static int		hsa_unbind(void);
static int		hsa_idr_find_ptr(void);
static long		ioctls(struct file *, unsigned int, unsigned long);

static DEFINE_IDR(ids);
static DEFINE_MUTEX(ids_lock);
static DEFINE_MUTEX(mem_lock);
static DEFINE_MUTEX(ioctl_lock);

static u64 init = 0, bind = 0, add_s = 0, remove_s = 0, unbind = 0, free = 0; 

static struct file_operations fops =
{
	.open = dev_open,
	.read = dev_read,
	.release = dev_release,
	.mmap = dev_mmap,
	.unlocked_ioctl = ioctls,
};

/*
 * Provides a directory path for device driver file
 */
static char *hsa_devnode(struct device *dev, umode_t *mode){
	
	return kasprintf(GFP_KERNEL, "hsa/%s", dev_name(dev));
}


static int vaddr2paddr(unsigned long vaddr, u64 *paddr)
{
        pgd_t *pgd;
        pud_t *pud;
        pmd_t *pmd;
        pte_t *pte;
        unsigned long pa = 0; 
        unsigned long page_addr = 0; 
        unsigned long page_offset = 0; 

	//printk("vaddr: 0x%lx\n", vaddr);
	
        pgd = pgd_offset(current->mm, vaddr);
        //printk("pgd_val = 0x%lx\n", pgd_val(*pgd));
        //printk("pgd_index = %lu\n", pgd_index(vaddr));
        if (pgd_none(*pgd)) {
                printk("not mapped in pgd\n");
                return -1;
        }
        //printk("pgd: 0x%llx\n", pgd_val(*pgd));  

        pud = pud_offset(pgd, vaddr);
        //printk("pud_val = 0x%lx\n", pud_val(*pud));
        if (pud_none(*pud)) {
                printk("not mapped in pud\n");
                return -1;
        }
       	//printk("pud: 0x%llx\n", pud_val(*pud));  

        pmd = pmd_offset(pud, vaddr);
        //printk("pmd_val = 0x%lx\n", pmd_val(*pmd));
        //printk("pmd_index = %lu\n", pmd_index(vaddr));
        if (pmd_none(*pmd)) {
                printk("not mapped in pmd\n");
                return -1;
        }
        //printk("pmd: 0x%llx\n", pmd_val(*pmd));  

        pte = pte_offset_kernel(pmd, vaddr);
        //printk("pte_val = 0x%lx\n", pte_val(*pte));
        //printk("pte_index = %lu\n", pte_index(vaddr));
        if (pte_none(*pte)) {
                printk("not mapped in pte\n");
                return -1;
        }
        //printk("pte: 0x%llx\n", pte_val(*pte));  


        /* Page frame physical address mechanism | offset */
        page_addr = pte_val(*pte) & PAGE_MASK;
	page_addr &= ((u64)1 << 48) - 1; 
        page_offset = vaddr & ~PAGE_MASK;
        pa = page_addr | page_offset;
        *paddr = pa;
	//printk("page_addr = %lx, page_offset = %lx\n", page_addr, page_offset);
          //  printk("vaddr = %lx, paddr = %lx\n", vaddr, pa);
        return 0;
}

static long ioctls(struct file *filep, unsigned int cmd, unsigned long arg) {

	u64 paddr;
	u64 vaddr;
	unsigned long order;
	int ret;
	int i,j;
	int id = 1;
	struct timespec	t0, t1;
	u16 sid [] = {0x200, 0x201, 0x202, 0x203, 0x204, 0x205, 0x206,
			0x207, 0x208, 0x209, 0x20a, 0x20b, 0x20c, 0x20d,
			0x20e, 0x20f};	
	switch (cmd) {

		case HSA_PA:
			mutex_lock(&ioctl_lock);
			if (copy_from_user(&vaddr, (void __user *)arg, sizeof(vaddr))) {
				printk("copy from user in HSA_PA failed\n");
				return -EFAULT;
			}
			
			if (vaddr2paddr(vaddr, &paddr)){
				printk("ioctl HSA_PA failed\n");
				return -ENOMEM;
			}
			
			if (copy_to_user((void __user *)arg, &paddr, sizeof(paddr))) {
				printk("copy to user in HSA_PA failed\n");
				return -EFAULT;
			}
			mutex_unlock(&ioctl_lock);
			break;
		case HSA_FREE:
			if (copy_from_user(&paddr, (void __user *)arg, sizeof(paddr))) {
				printk("copy from user in HSA_PA failed\n");
				return -EFAULT;
			}
			if (copy_from_user(&order, ((u64 __user *)arg)+1, sizeof(order))) {
				printk("copy from user in HSA_PA failed\n");
				return -EFAULT;
			}
			free_pages((unsigned long)__va((void*)paddr), order);
			break;
		case HSA_BIND:
			ret = hsa_bind();
			if (ret < 0) {
				pr_err("hsa_bind failed \n");
				return ret;
			}
			break;
		case HSA_BENCH:
			if (copy_from_user(&i, (void __user *)arg, sizeof(i))) {
				printk("copy from user in HSA_BENCH failed\n");
				return -EFAULT;
			}
			arm_smmu_free_streams(SID_BASE);		
			for (j = 0; j < 15; ++j) {	
				getnstimeofday(&t0);
				ret = arm_smmu_init_streams(SID_BASE, 16);
				getnstimeofday(&t1);
				init += (t1.tv_sec*NANO + t1.tv_nsec) - (t0.tv_sec*NANO + t0.tv_nsec);
				
				getnstimeofday(&t0);
				ret = arm_smmu_bind_pasid(id);
				getnstimeofday(&t1);
				bind += (t1.tv_sec*NANO + t1.tv_nsec) - (t0.tv_sec*NANO + t0.tv_nsec);

				getnstimeofday(&t0);
				ret = arm_smmu_add_streams(id, sid, i);
				getnstimeofday(&t1);
				add_s += (t1.tv_sec*NANO + t1.tv_nsec) - (t0.tv_sec*NANO + t0.tv_nsec);
				
				getnstimeofday(&t0);
				arm_smmu_remove_streams(id, sid, i);
				getnstimeofday(&t1);
				remove_s += (t1.tv_sec*NANO + t1.tv_nsec) - (t0.tv_sec*NANO + t0.tv_nsec);
				
				getnstimeofday(&t0);
				arm_smmu_unbind_pasid(id);
				getnstimeofday(&t1);
				unbind += (t1.tv_sec*NANO + t1.tv_nsec) - (t0.tv_sec*NANO + t0.tv_nsec);
				
				getnstimeofday(&t0);
				arm_smmu_free_streams(SID_BASE);
				getnstimeofday(&t1);
				free += (t1.tv_sec*NANO + t1.tv_nsec) - (t0.tv_sec*NANO + t0.tv_nsec);
	
			}
			printk(KERN_INFO "%d, %llu, %llu, %llu, %llu, %llu, %llu\n", i,
					init/15, bind/15, add_s/15, remove_s/15, unbind/15, free/15);
			init = 0, bind = 0; add_s = 0, remove_s = 0, unbind = 0, free = 0;
			arm_smmu_init_streams(SID_BASE, 16);
			return 0;
			break;
		default:
			return -ENOTTY;
	}

	return 0;

}

static int __init hsachar_init(void){
	
	int ret = 0;

	//pr_info("HSA: Initializing the hsaChar device\n");

	// Try to dynamically allocate a major number for the device -- more difficult but worth it
	majorNumber = register_chrdev(0, MISC_NAME, &fops);
	if (majorNumber<0){
		pr_err("hsaChar failed to register a major number\n");
		return majorNumber;
	}
	//pr_info("hsaChar: registered correctly with major number %d\n", majorNumber);

	// Register the device class
	class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(class)){                
		unregister_chrdev(majorNumber, MISC_NAME);
		pr_err("Failed to register device class\n");
		return PTR_ERR(class);          
	}
	class->devnode = hsa_devnode;
	//pr_info("hsaChar: device class registered correctly\n");
	
	// Register the device driver
	device = device_create(class, NULL, MKDEV(majorNumber, MISC_MINOR), NULL, MISC_NAME);
	if (IS_ERR(device)){               
		class_destroy(class);         
		unregister_chrdev(majorNumber, MISC_NAME);
		pr_err("Failed to create the device\n");
		return PTR_ERR(device);
	}
	//pr_info("hsaChar: device class created correctly\n"); 

	// Set SIDs in smmu driver and initialize stream contexts
	ret = arm_smmu_init_streams(SID_BASE, 16);
	if (ret < 0) {
		pr_err("init streams failed\n");
		return ret;
	}

	return 0;
}


static void __exit hsachar_exit(void){
	
	device_destroy(class, MKDEV(majorNumber, MISC_MINOR));     
	device_destroy(class, MKDEV(majorNumber, DEVICE_MINOR));
	//class_unregister(class);                          
	class_destroy(class);                            
	unregister_chrdev(majorNumber, MISC_NAME);    
	unregister_chrdev(majorNumber, DEVICE_NAME);
	
	arm_smmu_free_streams(SID_BASE);
	//pr_info("hsaChar: Goodbye from the LKM!\n");
}


static int dev_mmap(struct file *fileptr, struct vm_area_struct *vma) {

	phys_addr_t paddr;
	unsigned long kernel_addr = 1;
	int err = 0;

	mutex_lock(&mem_lock);

	kernel_addr = __get_free_pages(__GFP_DMA, get_order(vma->vm_end - vma->vm_start));
	if (!kernel_addr) {
		printk(KERN_ERR "hsaChar: mmap not enough memory\n");
		return -ENOMEM;
	}
	paddr = __virt_to_phys(kernel_addr);

	//vma->vm_flags |= VM_DONTCOPY | VM_DONTEXPAND | VM_NORESERVE | VM_DONTDUMP | VM_PFNMAP;
	//vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
	
	err = remap_pfn_range(vma, vma->vm_start, paddr >> PAGE_SHIFT, 
				  vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if(err < 0){
		printk(KERN_ERR "hsaChar: remapping of signal page frames to user space failed\n");
	}

	mutex_unlock(&mem_lock);
	return err;
}

static int dev_open(struct inode *inodep, struct file *filep){
	
	int ret = 0;
	
	numberOpens++;
	pr_info("hsaChar: Device has been opened %d time(s), currently from pid %d, with axiid: %d\n", 
			numberOpens, task_pid_nr(current), ret);

	return 0;
}

/*
 * Userland program can read from /dev/hsa/hsa file and get its axi id
 */
static ssize_t dev_read(struct file *filep, char __user *buffer, size_t len, loff_t *offset){

	int id;
	size_t unread, size;
	
	mutex_lock(&ids_lock);
	id = idr_alloc(&ids, current, AXIID_START, AXIID_END, GFP_KERNEL);
	mutex_unlock(&ids_lock);
	PDEBUG("%s, id: %d\n", __func__, id);
	
	if (id < 0) {
		pr_err("%s, idr_alloc failed\n", __func__);
		return id;
	}
	
	size = sizeof(id);
	PDEBUG("%s, len: %lu\n", __func__, len);
	unread = copy_to_user((void *) buffer, &id, size);
	return size;
}


static int dev_release(struct inode *inodep, struct file *filep){
	
	int ret;
	
	ret = hsa_unbind();
	if (ret) {
		pr_warning("hsa_unbind failed\n");
		return ret;
	}

	//pr_info("hsaChar: Device successfully closed\n");
	return 0;
}

/*
 * Finds a free axi id and adds this process' task to it
 * @return positive id  on success, negative values for failure
 */
static int hsa_bind() {

	int id, ret;
	u16 sid;

	mutex_lock(&ids_lock);
	id = hsa_idr_find_ptr();
	mutex_unlock(&ids_lock);
	

	sid = SID_BASE + id;
	ret = arm_smmu_bind_pasid(id);
	if (ret) {
		pr_err("%s, arm_smmu_bind_pasid failed\n", __func__);
		idr_remove(&ids, id);
		return ret;
	}

	ret = arm_smmu_add_streams(id, &sid, 1);
	if (ret) {
		pr_err("%s, arm_smmu_add_stream failed\n", __func__);
		idr_remove(&ids, id);
		return ret;
	}

	return id;
}

/*
 * Decouples axi id from current process and removes according stream from smmu
 * @return 0 on success, negative value on error
 */
static int hsa_unbind() {

	int id;

	mutex_lock(&ids_lock);
	id = hsa_idr_find_ptr();
	if (id < 0) {
		pr_err("id of task doesn't exist\n");
		goto out_err;
	}
	
	arm_smmu_unbind_pasid(id);
	idr_remove(&ids, id);
	mutex_unlock(&ids_lock);

	return 0;

out_err:
	mutex_unlock(&ids_lock);
	return -EFAULT;
}

/*
 * Finds for current process the correct axi id
 * @return valid id or negative error
 */
static int hsa_idr_find_ptr(){

	int id;
	struct task_struct *task;

	idr_for_each_entry(&ids, task, id) {
		if (task == current)
			return id;
	}

	return -EINVAL;
}

module_init(hsachar_init);
module_exit(hsachar_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lukas Liebischer");
MODULE_DESCRIPTION("HSA SMMU");
MODULE_VERSION("0.1");
