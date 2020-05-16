#define pr_fmt(fmt) "arm-smmu: " fmt

#include <linux/acpi.h>
#include <linux/acpi_iort.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/dma-iommu.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/hsa-smmu.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mmu_notifier.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_iommu.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/amba/bus.h>

#include "io-pgtable.h"
#include "arm-smmu-regs.h"

//#define SMMU_DEBUG

#undef PDEBUG             /* undef it, just in case */
#ifdef SMMU_DEBUG
#define PDEBUG(fmt, args...)		printk( KERN_DEBUG "hsa-smmu: " fmt, ## args)
#else
#define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#undef PDEBUGG
#define PDEBUGG(fmt, args...) /* nothing: it's a placeholder */




#define ARM_MMU500_ACTLR_CPRE		(1 << 1)

#define ARM_MMU500_ACR_CACHE_LOCK	(1 << 26)
#define ARM_MMU500_ACR_SMTNMB_TLBEN	(1 << 8)

#define TLB_LOOP_TIMEOUT		1000000	/* 1s! */
#define TLB_SPIN_COUNT			10

/* Maximum number of context banks per SMMU */
#define ARM_SMMU_MAX_CBS		128

/* SMMU global address space */
#define ARM_SMMU_GR0(smmu)		((smmu)->base)
#define ARM_SMMU_GR1(smmu)		((smmu)->base + (1 << (smmu)->pgshift))

#define INVALID_IRPTNDX			0xff
#define INVALID_PASID			0xffffffff
/*
 * SMMU global address space with conditional offset to access secure
 * aliases of non-secure registers (e.g. nsCR0: 0x400, nsGFSR: 0x448,
 * nsGFSYNR0: 0x450)
 */
#define ARM_SMMU_GR0_NS(smmu)						\
	((smmu)->base +							\
		((smmu->options & ARM_SMMU_OPT_SECURE_CFG_ACCESS)	\
			? 0x400 : 0))

/*
 * Some 64-bit registers only make sense to write atomically, but in such
 * cases all the data relevant to AArch32 formats lies within the lower word,
 * therefore this actually makes more sense than it might first appear.
 */
#ifdef CONFIG_64BIT
#define smmu_write_atomic_lq		writeq_relaxed
#else
#define smmu_write_atomic_lq		writel_relaxed
#endif

/* Translation context bank */
#define ARM_SMMU_CB(smmu, n)	((smmu)->cb_base + ((n) << (smmu)->pgshift))


enum arm_smmu_arch_version {
	ARM_SMMU_V1,
	ARM_SMMU_V1_64K,
	ARM_SMMU_V2,
};

enum arm_smmu_implementation {
	GENERIC_SMMU,
	ARM_MMU500,
	CAVIUM_SMMUV2,
};

/* Stream to context register 
 * points stream mapping table entry to context bank 
 */
struct arm_smmu_s2cr {
	int				count;
	enum arm_smmu_s2cr_type		type;
	enum arm_smmu_s2cr_privcfg	privcfg;
	u8				cbndx;
};

#define s2cr_init_val (struct arm_smmu_s2cr){				\
	.type =  S2CR_TYPE_BYPASS,	\
}

/* Stream match register
 * Matches incoming SID to stream mapping table entry
 */
struct arm_smmu_smr {
	u16				mask;
	u16				sid;
	bool				valid;
	struct mutex			smr_mutex;
};

/* Context bank
 * Holds info about page table
 */
struct arm_smmu_cb {
	u64				ttbr[2];
	u32				tcr[2];
	u32				mair[2];
	struct arm_smmu_cfg		*cfg;
	struct mutex			cb_mutex;
	bool				used;
};


struct sid_range {
	struct list_head list;
	u16 base;
	u16 num_sids;
};

struct  idx {
	struct list_head list;
	u8 idx;
	u16 sid;
};


#define NUM_CONTEXTS			16
#define INVALID_SMENDX			-1
#define INVALID_SID			-1u
#define AXIID_BITS			6


struct arm_smmu_device {
	struct device			*dev;

	void __iomem			*base;
	void __iomem			*cb_base;
	unsigned long			pgshift;

#define ARM_SMMU_FEAT_COHERENT_WALK	(1 << 0)
#define ARM_SMMU_FEAT_STREAM_MATCH	(1 << 1)
#define ARM_SMMU_FEAT_TRANS_S1		(1 << 2)
#define ARM_SMMU_FEAT_TRANS_S2		(1 << 3)
#define ARM_SMMU_FEAT_TRANS_NESTED	(1 << 4)
#define ARM_SMMU_FEAT_TRANS_OPS		(1 << 5)
#define ARM_SMMU_FEAT_VMID16		(1 << 6)
#define ARM_SMMU_FEAT_FMT_AARCH64_4K	(1 << 7)
#define ARM_SMMU_FEAT_FMT_AARCH64_16K	(1 << 8)
#define ARM_SMMU_FEAT_FMT_AARCH64_64K	(1 << 9)
#define ARM_SMMU_FEAT_FMT_AARCH32_L	(1 << 10)
#define ARM_SMMU_FEAT_FMT_AARCH32_S	(1 << 11)
#define ARM_SMMU_FEAT_EXIDS		(1 << 12)
	u32				features;

#define ARM_SMMU_OPT_SECURE_CFG_ACCESS (1 << 0)
	u32				options;
	enum arm_smmu_arch_version	version;
	enum arm_smmu_implementation	model;

	u32				num_context_banks;
	u32				num_s2_context_banks;
	DECLARE_BITMAP(context_map, ARM_SMMU_MAX_CBS);
	struct arm_smmu_cb		*cbs;
	atomic_t			irptndx;

	struct list_head		sid_ranges_head;
	u32				num_mapping_groups;
	atomic_t			num_sids_left;
	u16				streamid_mask;
	u16				smr_mask_mask;
	u8				axiid_mask;
	
	/* smrs + s2crs form together the stream mapping table
	 * idx is the index pointing to a specific entry in this table
	 */
	struct arm_smmu_smr		*smrs;
	struct arm_smmu_s2cr		*s2crs;
	s16				*smendx;
	u16				*sids;
	struct arm_smmu_domain		*domains;

	unsigned long			va_size;
	unsigned long			ipa_size;
	unsigned long			pa_size;
	unsigned long			pgsize_bitmap;

	u32				num_global_irqs;
	u32				num_context_irqs;
	unsigned int			*irqs;

	u32				cavium_id_base; /* Specific to Cavium */

	spinlock_t			global_sync_lock;

};

enum arm_smmu_context_fmt {
	ARM_SMMU_CTX_FMT_NONE,
	ARM_SMMU_CTX_FMT_AARCH64,
	ARM_SMMU_CTX_FMT_AARCH32_L,
	ARM_SMMU_CTX_FMT_AARCH32_S,
};

struct arm_smmu_cfg {
	u8				cbndx;
	u8				irptndx;
	union {
		u16			asid;
		u16			vmid;
	};
	u32				cbar;
	enum arm_smmu_context_fmt	fmt;
	struct task_struct		*task;
	struct mm_struct		*mm;
};

enum arm_smmu_domain_stage {
	ARM_SMMU_DOMAIN_S1 = 0,
	ARM_SMMU_DOMAIN_S2,
	ARM_SMMU_DOMAIN_NESTED,
	ARM_SMMU_DOMAIN_BYPASS,
};

/* Each domain is tied to one single context bank
 * holds info about SIDs of all streams wanting to 
 * access the context bank
 * Also holds info about in which stream mapping
 * table entries the context bank is referenced
 */
struct arm_smmu_domain {
	struct arm_smmu_device		*smmu;
	struct io_pgtable_cfg		*pgtbl_cfg;
	struct arm_smmu_cfg		cfg;
	enum arm_smmu_domain_stage	stage;
	struct mutex			domain_mutex; /* Protects smmu pointer */
	spinlock_t			cb_lock; /* Serialises ATS1* ops and TLB syncs */
	struct list_head		idxs;
	u16				num_idxs;
	u32				pasid;
	struct mmu_notifier		mn;
};

struct arm_smmu_option_prop {
	u32 opt;
	const char *prop;
};

static struct platform_driver arm_smmu_driver;

static struct arm_smmu_device *smmu_device;

static atomic_t cavium_smmu_context_count = ATOMIC_INIT(0);

static bool using_legacy_binding, using_generic_binding;


static struct arm_smmu_option_prop arm_smmu_options[] = {
	{ ARM_SMMU_OPT_SECURE_CFG_ACCESS, "calxeda,smmu-secure-config-access" },
	{ 0, NULL},
};


struct arm_smmu_match_data {
	enum arm_smmu_arch_version version;
	enum arm_smmu_implementation model;
};

#define ARM_SMMU_MATCH_DATA(name, ver, imp)	\
static struct arm_smmu_match_data name = { .version = ver, .model = imp }

ARM_SMMU_MATCH_DATA(arm_smmu, ARM_SMMU_V2, ARM_MMU500);

static const struct of_device_id arm_smmu_of_match[] = {
	{ .compatible = "i3,smmu-1", .data = &arm_smmu },
	{ },
};
MODULE_DEVICE_TABLE(of, arm_smmu_of_match);


void arm_64_lpae_pgtbl_cfg(struct io_pgtable_cfg *cfg);
static int arm_smmu_init_stream_context(struct arm_smmu_domain *, u8);
static int arm_smmu_master_write_sme(struct arm_smmu_device *, u8);



/* Wait for any pending TLB invalidations to complete */
static void __arm_smmu_tlb_sync(struct arm_smmu_device *smmu,
				void __iomem *sync, void __iomem *status)
{
	unsigned int spin_cnt, delay;

	writel_relaxed(0, sync);
	for (delay = 1; delay < TLB_LOOP_TIMEOUT; delay *= 2) {
		for (spin_cnt = TLB_SPIN_COUNT; spin_cnt > 0; spin_cnt--) {
			if (!(readl_relaxed(status) & sTLBGSTATUS_GSACTIVE))
				return;
			cpu_relax();
		}
		udelay(delay);
	}
	dev_err_ratelimited(smmu->dev,
			    "TLB sync timed out -- SMMU may be deadlocked\n");
}

static void arm_smmu_tlb_sync_global(struct arm_smmu_device *smmu)
{
	void __iomem *base = ARM_SMMU_GR0(smmu);
	unsigned long flags;

	spin_lock_irqsave(&smmu->global_sync_lock, flags);
	__arm_smmu_tlb_sync(smmu, base + ARM_SMMU_GR0_sTLBGSYNC,
			    base + ARM_SMMU_GR0_sTLBGSTATUS);
	spin_unlock_irqrestore(&smmu->global_sync_lock, flags);
}

static void arm_smmu_tlb_sync_context(void *cookie)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *base = ARM_SMMU_CB(smmu, smmu_domain->cfg.cbndx);
	unsigned long flags;

	spin_lock_irqsave(&smmu_domain->cb_lock, flags);
	__arm_smmu_tlb_sync(smmu, base + ARM_SMMU_CB_TLBSYNC,
			    base + ARM_SMMU_CB_TLBSTATUS);
	spin_unlock_irqrestore(&smmu_domain->cb_lock, flags);
}

static void arm_smmu_tlb_sync_cb(struct arm_smmu_device *smmu, u8 cbndx)
{
	void __iomem *base = ARM_SMMU_CB(smmu, cbndx);

	__arm_smmu_tlb_sync(smmu, base + ARM_SMMU_CB_TLBSYNC,
			    base + ARM_SMMU_CB_TLBSTATUS);
}

static void arm_smmu_tlb_inv_context_s1(void *cookie)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	void __iomem *base = ARM_SMMU_CB(smmu_domain->smmu, cfg->cbndx);

	writel_relaxed(cfg->asid, base + ARM_SMMU_CB_S1_TLBIASID);
	arm_smmu_tlb_sync_context(cookie);
}

/* Invalidates complete stage 1 TLB for a specific context bank
 * When called, cb_mutex must already be locked
 */
static void arm_smmu_tlb_inv_all_s1(struct arm_smmu_device *smmu, u8 cbndx)
{
	void __iomem *base = ARM_SMMU_CB(smmu, cbndx);

	writel_relaxed(0xffff, base + ARM_SMMU_CB_S1_TLBIALL);
	arm_smmu_tlb_sync_cb(smmu, cbndx);
}

/* Invalidates a range of virtual addresses
 * @param va start address
 * @param size length of va range
 * @param granule number of addresses which are going to 
 * invalidated at once
 * @param leaf indicates top level pte
 * @param domain ptr to smmu_domain associated with required
 * context bank
 */
static void arm_smmu_inv_range_s1(unsigned long va, size_t size,
				  size_t granule, bool leaf, 
				  struct arm_smmu_domain *domain)
{
	struct arm_smmu_cfg *cfg = &domain->cfg;
	void __iomem *reg = ARM_SMMU_CB(domain->smmu, cfg->cbndx);

	reg += leaf ? ARM_SMMU_CB_S1_TLBIVAL : ARM_SMMU_CB_S1_TLBIVA;

	va >>= 12;
	va |= (u64)cfg->asid << 48;
	do {	
		writeq_relaxed(va, reg);
		va += granule >> 12;
	} while (size -= granule);

	//sync is for inval_va not needed, I guess....
	//arm_smmu_tlb_sync_cb(domain->smmu, cfg->cbndx);
}

/* called from mmu_notifier_unregister
 * No further actions required, because TLB is cleaned explicitly 
 * in unbind()
 */
static void arm_svm_release(struct mmu_notifier *mn, struct mm_struct *mm) {

	struct arm_smmu_domain *domain;

	domain = container_of(mn, struct arm_smmu_domain, mn);
	PDEBUG("%s passed mm: 0x%llx, mm of mn: 0x%llx\n", 
			__func__, (u64)mm, (u64)domain->cfg.mm);
}

/* Called by a memory notifier
 * @param mn memory notifier which called this function
 * @param mm previously with mn registered mm_struct
 * @param start first virtual address
 * @param end last virtual address in range
 */
static void arm_svm_inv_range(struct mmu_notifier *mn, struct mm_struct *mm,
				unsigned long start, unsigned long end)
{

	struct arm_smmu_domain *domain;

	domain = container_of(mn, struct arm_smmu_domain, mn);
	/* granularity of page_size, 
	 * No leaf pte
	 */
	arm_smmu_inv_range_s1(start, end-start, 1<<(domain->smmu->pgshift), false, domain);
}

/* Function pointers for memory notifier operations
 */
static struct mmu_notifier_ops mn_ops = {
	.release = arm_svm_release,
	.invalidate_range = arm_svm_inv_range,
};

/* Compares recoreded fault syndromes (tried to write, read, ...)
 * to vma flags to find out if there are any access errors
 */
static bool access_error(struct vm_area_struct *vma, u32 fsynr)
{
	unsigned long requested = 0;

	if (fsynr & FSYNR0_IND)
		requested |= VM_EXEC;

	if (!(fsynr & FSYNR0_WNR))
		requested |= VM_READ;

	if (fsynr & FSYNR0_WNR)
		requested |= VM_WRITE;

	return (requested & ~vma->vm_flags) != 0;
}

/* Handles page faults
 * To be executed only in process context (might sleep)
 * @param mm of process which belongs to the following address
 * @param address which caused the page fault
 * @param fsynr Fault syndrome register which was recorded when 
 * page fault occured
 * @return 0 on success, < 0 on failure
 */
static int do_fault(struct mm_struct *mm, u64 address, u32 fsynr)
{
	struct vm_area_struct *vma;
	int ret = 0;
	unsigned int flags = 0;

	/* set flags to handle fault according to recorded fsynr */
	if (!(fsynr & FSYNR0_PNU))
		flags |= FAULT_FLAG_USER;
	if (fsynr & FSYNR0_WNR)
		flags |= FAULT_FLAG_WRITE;
	flags |= FAULT_FLAG_REMOTE;
	
	down_read(&mm->mmap_sem);
	vma = find_extend_vma(mm, address);
	if (!vma || address < vma->vm_start) {
		/* failed to get a vma in the right range */
		pr_err("hsa-smmu: no VMA found\n");
		ret = -EFAULT;
		goto out;
	}

	/* Check if we have the right permissions on the vma */
	if (access_error(vma, fsynr)) {
		ret = -EACCES;
		goto out;
	}

	ret = handle_mm_fault(vma, address, flags);
	if (ret & VM_FAULT_ERROR)
		ret = -EFAULT;

out:
	up_read(&mm->mmap_sem);
	return ret;
}

/* Top half Context bank interrupt handler 
 * for hard interrupt context
 */
static irqreturn_t handler(int irq, void *dev) {

	u32 fsr;
	struct arm_smmu_domain *smmu_domain = dev;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *cb_base;
	
	cb_base = ARM_SMMU_CB(smmu, cfg->cbndx);
	fsr = readl_relaxed(cb_base + ARM_SMMU_CB_FSR);
	
	if (!(fsr & FSR_FAULT))
		return IRQ_NONE;

	return IRQ_WAKE_THREAD;
}

/* Bottom half of context bank interrupt handler executed
 * in process context if handler() returns IRQ_WAKE_THREAD
 */
static irqreturn_t arm_smmu_context_fault(int irq, void *dev)
{
	u32 fsr, fsynr;
	unsigned long iova;
	struct arm_smmu_domain *smmu_domain = dev;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *cb_base;
	int ret = -EIO;

	cb_base = ARM_SMMU_CB(smmu, cfg->cbndx);
	fsr = readl_relaxed(cb_base + ARM_SMMU_CB_FSR);
	fsynr = readl_relaxed(cb_base + ARM_SMMU_CB_FSYNR0);
	iova = readq_relaxed(cb_base + ARM_SMMU_CB_FAR);

	/* handle page fault if either a translation fault occured
	 * or if an access flag / permission fault in level 3 of the
	 * translation table walk occured
	 */
	if ( (fsr & FSR_TF) || (((fsr & FSR_AFF) || (fsr & FSR_PF)) && (fsynr & 0x3)) )
		ret = do_fault(cfg->mm, iova, fsynr);

	/*clear fsr*/
	writel(fsr, cb_base + ARM_SMMU_CB_FSR);

	if (ret < 0) {
		dev_err_ratelimited(smmu->dev, 
				"a context fault occurred with error code %d!\n", ret);
		dev_err_ratelimited(smmu->dev, "fsr : 0x%x, cb: %d\n", fsr, cfg->cbndx);
		dev_err_ratelimited(smmu->dev, "fsynr: 0x%x\n", fsynr);
		dev_err_ratelimited(smmu->dev, "iova: 0x%lx\n", iova);
		dev_err_ratelimited(smmu->dev, "abort transaction\n");

		/*abort stalled transaction*/
		writel(0x1, cb_base + ARM_SMMU_CB_RESUME);

	} else {
		/*retry stalled transaction*/
		writel(0x0, cb_base + ARM_SMMU_CB_RESUME);
	}

	return IRQ_HANDLED;
}

/* A simple top half interrupt handler */
static irqreturn_t arm_smmu_global_fault(int irq, void *dev)
{
	u32 gfsr, gfsynr0, gfsynr1, gfsynr2;
	struct arm_smmu_device *smmu = dev;
	void __iomem *gr0_base = ARM_SMMU_GR0_NS(smmu);

	gfsr = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSR);

	if (gfsr & 0x2) {
		return IRQ_NONE;
	}

	if (!gfsr)
		return IRQ_NONE;

	gfsynr0 = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSYNR0);
	gfsynr1 = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSYNR1);
	gfsynr2 = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSYNR2);

	dev_err_ratelimited(smmu->dev,
		"Unexpected global fault, this could be serious\n");
	dev_err_ratelimited(smmu->dev,
		"\tGFSR 0x%08x, GFSYNR0 0x%08x, GFSYNR1 0x%08x, GFSYNR2 0x%08x\n",
		gfsr, gfsynr0, gfsynr1, gfsynr2);

	writel(gfsr, gr0_base + ARM_SMMU_GR0_sGFSR);
	return IRQ_HANDLED;
}

static void arm_smmu_init_context_bank(struct arm_smmu_domain *smmu_domain,
				       struct io_pgtable_cfg *pgtbl_cfg)
{
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_cb *cb = &smmu_domain->smmu->cbs[cfg->cbndx];
	bool stage1 = cfg->cbar != CBAR_TYPE_S2_TRANS;

	cb->cfg = cfg;
	PDEBUG("%s: stage1: %s, cbndx: %d\n", __func__, stage1?"true":"false", cfg->cbndx);
	PDEBUG("%s, cbndx: %u, asid: %u\n", __func__, cfg->cbndx, cfg->asid);
	PDEBUG("%s, cbndx: 0x%x\n", __func__, cfg->cbndx);
	/* TTBCR */
	if (stage1) {
		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH32_S) {
			cb->tcr[0] = pgtbl_cfg->arm_v7s_cfg.tcr;
		} else {
			cb->tcr[0] = pgtbl_cfg->arm_lpae_s1_cfg.tcr;
			cb->tcr[1] = pgtbl_cfg->arm_lpae_s1_cfg.tcr >> 32;
			cb->tcr[1] |= TTBCR2_SEP_UPSTREAM;
			if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH64)
				cb->tcr[1] |= TTBCR2_AS;
			PDEBUG("%s: tcr: 0x%x\n", __func__, cb->tcr[0]);
			PDEBUG("%s: tcr2: 0x%x\n", __func__, cb->tcr[1]);
		}
	} else {
		cb->tcr[0] = pgtbl_cfg->arm_lpae_s2_cfg.vtcr;
	}

	/* TTBRs */
	if (stage1) {
		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH32_S) {
			cb->ttbr[0] = pgtbl_cfg->arm_v7s_cfg.ttbr[0];
			cb->ttbr[1] = pgtbl_cfg->arm_v7s_cfg.ttbr[1];
		} else {
			PDEBUG("%s: aarch64 ttbr, pgd phys: 0x%llx\n", __func__, virt_to_phys(current->mm->pgd));
			cb->ttbr[0] = virt_to_phys(current->mm->pgd);
			//
			//if (cfg->cbndx == 0)
			//	cb->ttbr[0] = 0;
			cb->ttbr[0] |= (u64)cfg->asid << TTBRn_ASID_SHIFT;
			cb->ttbr[1] = pgtbl_cfg->arm_lpae_s1_cfg.ttbr[1];
			cb->ttbr[1] |= (u64)cfg->asid << TTBRn_ASID_SHIFT;
			PDEBUG("%s: ttbr0: 0x%llx\n", __func__, cb->ttbr[0]);
			PDEBUG("%s: ttbr1: 0x%llx\n", __func__, cb->ttbr[1]);
			PDEBUG("%s: io ttbr: 0x%llx, virt: 0x%llx\n", __func__, 
					pgtbl_cfg->arm_lpae_s1_cfg.ttbr[0], 
					(u64) phys_to_virt(pgtbl_cfg->arm_lpae_s1_cfg.ttbr[0]));
		}
	} else {
		cb->ttbr[0] = pgtbl_cfg->arm_lpae_s2_cfg.vttbr;
	}

	/* MAIRs (stage-1 only) */
	if (stage1) {
		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH32_S) {
			cb->mair[0] = pgtbl_cfg->arm_v7s_cfg.prrr;
			cb->mair[1] = pgtbl_cfg->arm_v7s_cfg.nmrr;
		} else {
			cb->mair[0] = pgtbl_cfg->arm_lpae_s1_cfg.mair[0];
			cb->mair[1] = pgtbl_cfg->arm_lpae_s1_cfg.mair[1];
		}
	}
}

static void arm_smmu_write_smr(struct arm_smmu_device *smmu, int idx)
{
	struct arm_smmu_smr *smr = smmu->smrs + idx;
	u32 reg = smr->sid << SMR_ID_SHIFT | smr->mask << SMR_MASK_SHIFT;
	PDEBUG("%s, idx: %d, sid: 0x%x\n", __func__, idx, smr->sid);

	if (!(smmu->features & ARM_SMMU_FEAT_EXIDS) && smr->valid)
		reg |= SMR_VALID;
	PDEBUG("%s, smr: 0x%x\n", __func__, reg);
	writel_relaxed(reg, ARM_SMMU_GR0(smmu) + ARM_SMMU_GR0_SMR(idx));
}

static void arm_smmu_write_s2cr(struct arm_smmu_device *smmu, int idx)
{
	struct arm_smmu_s2cr *s2cr = smmu->s2crs + idx;
	u32 reg = (s2cr->type & S2CR_TYPE_MASK) << S2CR_TYPE_SHIFT |
		  (s2cr->cbndx & S2CR_CBNDX_MASK) << S2CR_CBNDX_SHIFT |
		  (s2cr->privcfg & S2CR_PRIVCFG_MASK) << S2CR_PRIVCFG_SHIFT;
	PDEBUG("%s, idx: %d, cbndx: %d\n", __func__, idx, s2cr->cbndx);

	if (smmu->features & ARM_SMMU_FEAT_EXIDS && smmu->smrs &&
	    smmu->smrs[idx].valid)
		reg |= S2CR_EXIDVALID;
	PDEBUG("%s, s2cr: 0x%x\n", __func__, reg);
	writel_relaxed(reg, ARM_SMMU_GR0(smmu) + ARM_SMMU_GR0_S2CR(idx));
}

static void arm_smmu_write_sme(struct arm_smmu_device *smmu, int idx)
{
	arm_smmu_write_s2cr(smmu, idx);
	if (smmu->smrs)
		arm_smmu_write_smr(smmu, idx);
}


static void arm_smmu_test_smr_masks(struct arm_smmu_device *smmu)
{
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);
	u32 smr;

	if (!smmu->smrs)
		return;

	/*
	 * SMR.ID bits may not be preserved if the corresponding MASK
	 * bits are set, so check each one separately. We can reject
	 * masters later if they try to claim IDs outside these masks.
	 */
	smr = smmu->streamid_mask << SMR_ID_SHIFT;
	writel_relaxed(smr, gr0_base + ARM_SMMU_GR0_SMR(0));
	smr = readl_relaxed(gr0_base + ARM_SMMU_GR0_SMR(0));
	smmu->streamid_mask = smr >> SMR_ID_SHIFT;

	smr = smmu->streamid_mask << SMR_MASK_SHIFT;
	writel_relaxed(smr, gr0_base + ARM_SMMU_GR0_SMR(0));
	smr = readl_relaxed(gr0_base + ARM_SMMU_GR0_SMR(0));
	smmu->smr_mask_mask = smr >> SMR_MASK_SHIFT;
}


static void arm_smmu_write_context_bank(struct arm_smmu_device *smmu, int idx)
{
	u32 reg;
	bool stage1;
	struct arm_smmu_cb *cb = &smmu->cbs[idx];
	struct arm_smmu_cfg *cfg = cb->cfg;
	void __iomem *cb_base, *gr1_base;

	PDEBUG("%s, cbndx: %d\n", __func__, idx);

	cb_base = ARM_SMMU_CB(smmu, idx);

	/* Unassigned context banks only need disabling */
	if (!cfg) {
		PDEBUG("%s, cbndx: %d disabling\n", __func__, idx);
		writel_relaxed(0, cb_base + ARM_SMMU_CB_SCTLR);
		return;
	}

	gr1_base = ARM_SMMU_GR1(smmu);
	stage1 = cfg->cbar != CBAR_TYPE_S2_TRANS;

	/* CBA2R */
	if (smmu->version > ARM_SMMU_V1) {
		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH64)
			reg = CBA2R_RW64_64BIT;
		else
			reg = CBA2R_RW64_32BIT;
		/* 16-bit VMIDs live in CBA2R */
		if (smmu->features & ARM_SMMU_FEAT_VMID16)
			reg |= cfg->vmid << CBA2R_VMID_SHIFT;
		
		PDEBUG("%s, cba2r: 0x%x\n", __func__, reg);
		writel_relaxed(reg, gr1_base + ARM_SMMU_GR1_CBA2R(idx));
	}

	/* CBAR */
	reg = cfg->cbar;
	if (smmu->version < ARM_SMMU_V2)
		reg |= cfg->irptndx << CBAR_IRPTNDX_SHIFT;

	/*
	 * Use the weakest shareability/memory types, so they are
	 * overridden by the ttbcr/pte.
	 */
	if (stage1) {
		reg |= (CBAR_S1_BPSHCFG_NSH << CBAR_S1_BPSHCFG_SHIFT) |
			(CBAR_S1_MEMATTR_WB << CBAR_S1_MEMATTR_SHIFT);
	} else if (!(smmu->features & ARM_SMMU_FEAT_VMID16)) {
		/* 8-bit VMIDs live in CBAR */
		reg |= cfg->vmid << CBAR_VMID_SHIFT;
	}
	PDEBUG("%s, cbar: 0x%x\n", __func__, reg);
	writel_relaxed(reg, gr1_base + ARM_SMMU_GR1_CBAR(idx));

	/*
	 * TTBCR
	 * We must write this before the TTBRs, since it determines the
	 * access behaviour of some fields (in particular, ASID[15:8]).
	 */
	if (stage1 && smmu->version > ARM_SMMU_V1)
		writel_relaxed(cb->tcr[1], cb_base + ARM_SMMU_CB_TTBCR2);
	writel_relaxed(cb->tcr[0], cb_base + ARM_SMMU_CB_TTBCR);

	/* TTBRs */
	if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH32_S) {
		writel_relaxed(cfg->asid, cb_base + ARM_SMMU_CB_CONTEXTIDR);
		writel_relaxed(cb->ttbr[0], cb_base + ARM_SMMU_CB_TTBR0);
		writel_relaxed(cb->ttbr[1], cb_base + ARM_SMMU_CB_TTBR1);
	} else {
		writeq_relaxed(cb->ttbr[0], cb_base + ARM_SMMU_CB_TTBR0);
		if (stage1)
			writeq_relaxed(cb->ttbr[1], cb_base + ARM_SMMU_CB_TTBR1);
	}

	/* MAIRs (stage-1 only) */
	if (stage1) {
		PDEBUG("%s: mair0: 0x%x\n", __func__, cb->mair[0]);
		PDEBUG("%s: mair1: 0x%x\n", __func__, cb->mair[1]);

		writel_relaxed(cb->mair[0], cb_base + ARM_SMMU_CB_S1_MAIR0);
		writel_relaxed(cb->mair[1], cb_base + ARM_SMMU_CB_S1_MAIR1);
	}

	/* SCTLR */
	reg = SCTLR_CFIE | SCTLR_CFRE | SCTLR_AFE | SCTLR_TRE | SCTLR_M;
	if (stage1)
		reg |= SCTLR_S1_ASIDPNE;
		//reg &= ~SCTLR_S1_ASIDPNE;
	if (IS_ENABLED(CONFIG_CPU_BIG_ENDIAN))
		reg |= SCTLR_E;

	reg |= SCTLR_CFCFG;
	PDEBUG("%s, sctlr: 0x%x, for cb: %d\n", __func__, reg, idx);
	writel_relaxed(reg, cb_base + ARM_SMMU_CB_SCTLR);

}




static void arm_smmu_device_reset(struct arm_smmu_device *smmu)
{
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);
	int i;
	u32 reg, major;

	/* clear global FSR */
	reg = readl_relaxed(ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sGFSR);
	writel(reg, ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sGFSR);

	/*
	 * Reset stream mapping groups: Initial values mark all SMRn as
	 * invalid and all S2CRn as bypass unless overridden.
	 */
	for (i = 0; i < smmu->num_mapping_groups; ++i)
		arm_smmu_write_sme(smmu, i);

	if (smmu->model == ARM_MMU500) {
		/*
		 * Before clearing ARM_MMU500_ACTLR_CPRE, need to
		 * clear CACHE_LOCK bit of ACR first. And, CACHE_LOCK
		 * bit is only present in MMU-500r2 onwards.
		 */
		reg = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID7);
		major = (reg >> ID7_MAJOR_SHIFT) & ID7_MAJOR_MASK;
		reg = readl_relaxed(gr0_base + ARM_SMMU_GR0_sACR);
		if (major >= 2)
			reg &= ~ARM_MMU500_ACR_CACHE_LOCK;
		/*
		 * Allow unmatched Stream IDs to allocate bypass
		 * TLB entries for reduced latency.
		 */
		reg |= ARM_MMU500_ACR_SMTNMB_TLBEN;
		writel_relaxed(reg, gr0_base + ARM_SMMU_GR0_sACR);
	}

	/* Make sure all context banks are disabled and clear CB_FSR  */
	for (i = 0; i < smmu->num_context_banks; ++i) {
		void __iomem *cb_base = ARM_SMMU_CB(smmu, i);

		arm_smmu_write_context_bank(smmu, i);
		writel_relaxed(FSR_FAULT, cb_base + ARM_SMMU_CB_FSR);
		/*
		 * Disable MMU-500's not-particularly-beneficial next-page
		 * prefetcher for the sake of errata #841119 and #826419.
		 */
		if (smmu->model == ARM_MMU500) {
			reg = readl_relaxed(cb_base + ARM_SMMU_CB_ACTLR);
			reg &= ~ARM_MMU500_ACTLR_CPRE;
			writel_relaxed(reg, cb_base + ARM_SMMU_CB_ACTLR);
		}
	}

	/* Invalidate the TLB, just in case */
	writel_relaxed(0, gr0_base + ARM_SMMU_GR0_TLBIALLH);
	writel_relaxed(0, gr0_base + ARM_SMMU_GR0_TLBIALLNSNH);

	reg = readl_relaxed(ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sCR0);

	/* Enable fault reporting */
	reg |= (sCR0_GFRE | sCR0_GFIE | sCR0_GCFGFRE | sCR0_GCFGFIE);

	/* Disable TLB broadcasting. */
	reg |= (sCR0_VMIDPNE | sCR0_PTM);
	//reg &= ~(sCR0_PTM);
	//reg &= ~(sCR0_VMIDPNE);

	/* Enable client access, handling unmatched streams as appropriate */
	reg &= ~sCR0_CLIENTPD;
	reg &= ~sCR0_USFCFG;

	/* Disable forced broadcasting */
	//reg &= ~sCR0_FB;
	reg |= sCR0_FB;

	/* Don't upgrade barriers */
	reg &= ~(sCR0_BSU_MASK << sCR0_BSU_SHIFT);

	if (smmu->features & ARM_SMMU_FEAT_VMID16)
		reg |= sCR0_VMID16EN;

	if (smmu->features & ARM_SMMU_FEAT_EXIDS)
		reg |= sCR0_EXIDENABLE;

	/* Push the button */
	arm_smmu_tlb_sync_global(smmu);
	writel(reg, ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sCR0);
}

static int arm_smmu_id_size_to_bits(int size)
{
	switch (size) {
	case 0:
		return 32;
	case 1:
		return 36;
	case 2:
		return 40;
	case 3:
		return 42;
	case 4:
		return 44;
	case 5:
	default:
		return 48;
	}
}

/* Allocates all domains for a given smmu device
 * @param smmu device
 * @retrun 0 on success, < 0 on failure
 */
int arm_smmu_domains_alloc(struct arm_smmu_device *smmu)
{
	struct arm_smmu_domain *domains = smmu->domains;
	int i;
	
	
	PDEBUG("%s\n", __func__);
	
	if (domains) {
		dev_err(smmu->dev, "Domains already exist\n");
		return -EEXIST;
	}

	domains = kcalloc(smmu->num_context_banks, sizeof(*domains), GFP_KERNEL);
	if (!domains) {
		dev_err(smmu->dev, "Allocation for domains failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < smmu->num_context_banks; ++i) {	
		domains[i].pasid = INVALID_PASID;
		mutex_init(&domains[i].domain_mutex);
		spin_lock_init(&domains[i].cb_lock);
		INIT_LIST_HEAD(&domains[i].idxs);
	}

	smmu->domains = domains;

	return 0;
}


static void parse_driver_options(struct arm_smmu_device *smmu)
{
	int i = 0;

	do {
		if (of_property_read_bool(smmu->dev->of_node,
						arm_smmu_options[i].prop)) {
			smmu->options |= arm_smmu_options[i].opt;
			dev_notice(smmu->dev, "option %s\n",
				arm_smmu_options[i].prop);
		}
	} while (arm_smmu_options[++i].opt);
}

static int arm_smmu_device_cfg_probe(struct arm_smmu_device *smmu)
{
	unsigned long size, i;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);
	u32 id;
	int ret;
	bool cttw_reg, cttw_fw = smmu->features & ARM_SMMU_FEAT_COHERENT_WALK;

	dev_notice(smmu->dev, "probing hardware configuration...\n");
	dev_notice(smmu->dev, "SMMUv%d with:\n",
			smmu->version == ARM_SMMU_V2 ? 2 : 1);

	/* ID0 */
	id = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID0);

	dev_notice(smmu->dev, "Broadcast TLB implemented: %d\n", (id & (1<<13)) > 0);

	if (id & ID0_S1TS) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_S1;
		dev_notice(smmu->dev, "\tstage 1 translation\n");
	}

	if (id & ID0_S2TS) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_S2;
		dev_notice(smmu->dev, "\tstage 2 translation\n");
	}

	if (id & ID0_NTS) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_NESTED;
		dev_notice(smmu->dev, "\tnested translation\n");
	}

	if (!(smmu->features &
		(ARM_SMMU_FEAT_TRANS_S1 | ARM_SMMU_FEAT_TRANS_S2))) {
		dev_err(smmu->dev, "\tno translation support!\n");
		return -ENODEV;
	}

	if ((id & ID0_S1TS) &&
		((smmu->version < ARM_SMMU_V2) || !(id & ID0_ATOSNS))) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_OPS;
		dev_notice(smmu->dev, "\taddress translation ops\n");
	}
	
	dev_notice(smmu->dev, "\tnum irpts: %d\n", (id >> ID0_NUMIRPT_SHIFT) & ID0_NUMIRPT_MASK );

	/*
	 * In order for DMA API calls to work properly, we must defer to what
	 * the FW says about coherency, regardless of what the hardware claims.
	 * Fortunately, this also opens up a workaround for systems where the
	 * ID register value has ended up configured incorrectly.
	 */
	cttw_reg = !!(id & ID0_CTTW);
	if (cttw_fw || cttw_reg)
		dev_notice(smmu->dev, "\t%scoherent table walk\n",
			   cttw_fw ? "" : "non-");
	if (cttw_fw != cttw_reg)
		dev_notice(smmu->dev,
			   "\t(IDR0.CTTW overridden by FW configuration)\n");

	/* Max. number of entries we have for stream matching/indexing */
	PDEBUG("%s, EXIDS %s\n", __func__, (id&ID0_EXIDS)?"true":"false");
	if (smmu->version == ARM_SMMU_V2 && id & ID0_EXIDS) {
		smmu->features |= ARM_SMMU_FEAT_EXIDS;
		size = 1 << 16;
	} else {
		size = 1 << ((id >> ID0_NUMSIDB_SHIFT) & ID0_NUMSIDB_MASK);
	}
	smmu->streamid_mask = size - 1;
	smmu->axiid_mask = (1 << AXIID_BITS)-1;
	if (id & ID0_SMS) {
		smmu->features |= ARM_SMMU_FEAT_STREAM_MATCH;
		size = (id >> ID0_NUMSMRG_SHIFT) & ID0_NUMSMRG_MASK;
		if (size == 0) {
			dev_err(smmu->dev,
				"stream-matching supported, but no SMRs present!\n");
			return -ENODEV;
		}

		/* Zero-initialised to mark as invalid */
		smmu->smrs = devm_kcalloc(smmu->dev, size, sizeof(*smmu->smrs),
					  GFP_KERNEL);
		if (!smmu->smrs)
			return -ENOMEM;

		for (i = 0; i < size; ++i)
			mutex_init(&(smmu->smrs[i].smr_mutex));

		dev_notice(smmu->dev,
			   "\tstream matching with %lu register groups", size);
	}
	/* s2cr->type == 0 means translation, so initialise explicitly */
	smmu->s2crs = devm_kmalloc_array(smmu->dev, size, sizeof(*smmu->s2crs),
					 GFP_KERNEL);
	if (!smmu->s2crs)
		return -ENOMEM;
	for (i = 0; i < size; i++)
		smmu->s2crs[i] = s2cr_init_val;

	smmu->num_mapping_groups = size;
	atomic_set(&smmu->num_sids_left, size);
	spin_lock_init(&smmu->global_sync_lock);

	if (smmu->version < ARM_SMMU_V2 || !(id & ID0_PTFS_NO_AARCH32)) {
		smmu->features |= ARM_SMMU_FEAT_FMT_AARCH32_L;
		if (!(id & ID0_PTFS_NO_AARCH32S))
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH32_S;
	}

	/* ID1 */
	id = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID1);
	smmu->pgshift = (id & ID1_PAGESIZE) ? 16 : 12;

	/* Check for HW dirty bit and access flag update*/
	dev_notice(smmu->dev, "\tHW DB+AF update: 0x%x\n", id & (2<<24));
		

	/* Check for size mismatch of SMMU address space from mapped region */
	size = 1 << (((id >> ID1_NUMPAGENDXB_SHIFT) & ID1_NUMPAGENDXB_MASK) + 1);
	size <<= smmu->pgshift;
	if (smmu->cb_base != gr0_base + size)
		dev_warn(smmu->dev,
			"SMMU address space size (0x%lx) differs from mapped region size (0x%tx)!\n",
			size * 2, (smmu->cb_base - gr0_base) * 2);

	smmu->num_s2_context_banks = (id >> ID1_NUMS2CB_SHIFT) & ID1_NUMS2CB_MASK;
	smmu->num_context_banks = (id >> ID1_NUMCB_SHIFT) & ID1_NUMCB_MASK;
	if (smmu->num_s2_context_banks > smmu->num_context_banks) {
		dev_err(smmu->dev, "impossible number of S2 context banks!\n");
		return -ENODEV;
	}
	dev_notice(smmu->dev, "\t%u context banks (%u stage-2 only)\n",
		   smmu->num_context_banks, smmu->num_s2_context_banks);
	/*
	 * Cavium CN88xx erratum #27704.
	 * Ensure ASID and VMID allocation is unique across all SMMUs in
	 * the system.
	 */
	if (smmu->model == CAVIUM_SMMUV2) {
		smmu->cavium_id_base =
			atomic_add_return(smmu->num_context_banks,
					  &cavium_smmu_context_count);
		smmu->cavium_id_base -= smmu->num_context_banks;
		dev_notice(smmu->dev, "\tenabling workaround for Cavium erratum 27704\n");
	}
	smmu->cbs = devm_kcalloc(smmu->dev, smmu->num_context_banks,
				 sizeof(*smmu->cbs), GFP_KERNEL);
	if (!smmu->cbs)
		return -ENOMEM;

	for (i = 0; i < smmu->num_context_banks; ++i)
		mutex_init(&smmu->cbs[i].cb_mutex);

	/* ID2 */
	id = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID2);
	size = arm_smmu_id_size_to_bits((id >> ID2_IAS_SHIFT) & ID2_IAS_MASK);
	/* Linux arm64 uses 40 bits for PA */
	smmu->ipa_size = 40;

	/* The output mask is also applied for bypass */
	size = arm_smmu_id_size_to_bits((id >> ID2_OAS_SHIFT) & ID2_OAS_MASK);
	smmu->pa_size = size;

	if (id & ID2_VMID16)
		smmu->features |= ARM_SMMU_FEAT_VMID16;

	/*
	 * What the page table walker can address actually depends on which
	 * descriptor format is in use, but since a) we don't know that yet,
	 * and b) it can vary per context bank, this will have to do...
	 */
	if (dma_set_mask_and_coherent(smmu->dev, DMA_BIT_MASK(size)))
		dev_warn(smmu->dev,
			 "failed to set DMA mask for table walker\n");

	if (smmu->version < ARM_SMMU_V2) {
		smmu->va_size = smmu->ipa_size;
		if (smmu->version == ARM_SMMU_V1_64K)
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH64_64K;
	} else {
		size = (id >> ID2_UBS_SHIFT) & ID2_UBS_MASK;
		
		/* In Linux arm64 VA size is always set to 39 bits */
		smmu->va_size = 39;
		if (id & ID2_PTFS_4K)
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH64_4K;
		if (id & ID2_PTFS_16K)
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH64_16K;
		if (id & ID2_PTFS_64K)
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH64_64K;
	}

	/* Now we've corralled the various formats, what'll it do? */
	if (smmu->features & ARM_SMMU_FEAT_FMT_AARCH32_S)
		smmu->pgsize_bitmap |= SZ_4K | SZ_64K | SZ_1M | SZ_16M;
	if (smmu->features &
	    (ARM_SMMU_FEAT_FMT_AARCH32_L | ARM_SMMU_FEAT_FMT_AARCH64_4K))
		smmu->pgsize_bitmap |= SZ_4K | SZ_2M | SZ_1G;
	if (smmu->features & ARM_SMMU_FEAT_FMT_AARCH64_16K)
		smmu->pgsize_bitmap |= SZ_16K | SZ_32M;
	if (smmu->features & ARM_SMMU_FEAT_FMT_AARCH64_64K)
		smmu->pgsize_bitmap |= SZ_64K | SZ_512M;

	dev_notice(smmu->dev, "\tSupported page sizes: 0x%08lx\n",
		   smmu->pgsize_bitmap);


	if (smmu->features & ARM_SMMU_FEAT_TRANS_S1)
		dev_notice(smmu->dev, "\tStage-1: %lu-bit VA -> %lu-bit IPA\n",
			   smmu->va_size, smmu->ipa_size);

	if (smmu->features & ARM_SMMU_FEAT_TRANS_S2)
		dev_notice(smmu->dev, "\tStage-2: %lu-bit IPA -> %lu-bit PA\n",
			   smmu->ipa_size, smmu->pa_size);
	
	INIT_LIST_HEAD(&smmu->sid_ranges_head);
	
	ret = arm_smmu_domains_alloc(smmu);
	if (ret) {
		dev_err(smmu_device->dev, "Domains alloc failed\n");
		return ret;
	}
	
	return 0;
}


static int arm_smmu_device_dt_probe(struct platform_device *pdev,
				    struct arm_smmu_device *smmu)
{
	const struct arm_smmu_match_data *data;
	struct device *dev = &pdev->dev;
	bool legacy_binding;

	PDEBUG("%s", __func__);
	if (of_property_read_u32(dev->of_node, "#global-interrupts",
				 &smmu->num_global_irqs)) {
		dev_err(dev, "missing #global-interrupts property\n");
		return -ENODEV;
	}

	data = of_device_get_match_data(dev);
	smmu->version = data->version;
	smmu->model = data->model;

	parse_driver_options(smmu);

	legacy_binding = of_find_property(dev->of_node, "mmu-masters", NULL);
	if (legacy_binding && !using_generic_binding) {
		if (!using_legacy_binding)
			pr_notice("deprecated \"mmu-masters\" DT property in use; DMA API support unavailable\n");
		using_legacy_binding = true;
		PDEBUG("%s, using legacy binding\n", __func__);
	} else if (!legacy_binding && !using_legacy_binding) {
		using_generic_binding = true;
		PDEBUG("%s, using generic binding\n", __func__);
	} else {
		dev_err(dev, "not probing due to mismatched DT properties\n");
		return -ENODEV;
	}

	if (of_dma_is_coherent(dev->of_node))
		smmu->features |= ARM_SMMU_FEAT_COHERENT_WALK;

	return 0;
}

/* Finds empty entry in stream mapping table (returned in idx) and 
 * initializes SMR
 * @param smmu holds pointer to SMRs
 * @param idx returns index of stream mapping table
 * @param sid SID to be registered in SMR[idx]
 * @return 0 on success, < 0 on failure
 */
static int arm_smmu_init_stream_match(struct arm_smmu_device *smmu, u8 *idx, u16 sid)
{
	struct arm_smmu_smr *smr = smmu->smrs;
	u16 mask;
	u8 i;
	atomic_t *num_sids_left = &smmu->num_sids_left;

	if (!smr) {
		dev_err(smmu->dev, "%s, smrs doesnt exist\n", __func__);
		return -EEXIST;
	}

	mask = sid >> SMR_MASK_SHIFT;

	for (i = 0; i < smmu->num_mapping_groups - atomic_read(num_sids_left); ++i) {	
		mutex_lock(&smr->smr_mutex);
		if (smmu->s2crs[i].count == 0 && !smr->valid) {
			smr->sid = sid;
			smr->mask = mask;
			smr->valid = true;	
			goto out;
		}
		mutex_unlock(&smr->smr_mutex);
		smr++;
	}

	dev_err(smmu->dev, "Stream match already exist\n");
	return -EFAULT;

out:
	smmu->s2crs[i].count++;
	mutex_unlock(&smr->smr_mutex);
	
	*idx = i;
	PDEBUG("%s, sid %d: 0x%x, valid: %d", __func__, i, sid, smr->valid);
	PDEBUG("%s, s2cr.count: %d at idx: %d, smr.sid: 0x%x, smr.valid: %d\n",
			__func__, smmu->s2crs[i].count, i, smr->sid, smr->valid);

	return 0;
}

/* Initializes S2CR
 * @param smmu_domain holds the index of the stream mapping table
 * (needed to get the context bank index)
 * @param idx index of stream mapping table
 * @return 0 on success, < 0 on failure
 */
static int arm_smmu_init_stream_context(struct arm_smmu_domain *smmu_domain, u8 idx)
{
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_s2cr *s2crs;
	u8 cbndx = smmu_domain->cfg.cbndx;
	enum arm_smmu_s2cr_type type;

	if (!smmu) {
		printk(KERN_ERR "%s, smmu doesn't exist\n", __func__);
		return -EEXIST;
	}
	
	s2crs = smmu->s2crs;
	
	if (!s2crs) {
		dev_err(smmu->dev, "s2cr doesn't exist\n");
		return -EEXIST;
	}

	if (smmu_domain->stage == ARM_SMMU_DOMAIN_BYPASS)
		type = S2CR_TYPE_BYPASS;
	else
		type = S2CR_TYPE_TRANS;

	PDEBUG("%s, s2crs.cbndx: %u, s2cr.type: %d\n", __func__, cbndx, type);

	s2crs[idx].type = type;
	s2crs[idx].privcfg = S2CR_PRIVCFG_DEFAULT;
	s2crs[idx].cbndx = cbndx;

	return 0;
}


/* Writes a stream mapping table entry to SMMU registers
 * Writes both SMR and S2CR
 * @param smmu device which hold SMR and S2CR registers
 * @param idx index of stream mapping table entry
 * @return 0 on success, < 0 on failure
 */
static int arm_smmu_master_write_sme(struct arm_smmu_device *smmu, u8 idx)
{
	struct arm_smmu_smr *smrs = smmu->smrs;

	mutex_lock(&smrs[idx].smr_mutex);
	
	if (!smrs[idx].valid) {
		dev_err(smmu->dev, "sme not valid\n");
		return -EFAULT;
	}
	PDEBUG("%s, write_sme, idx: %d, s2crs[i].cbndx: %d\n", __func__,
		idx, smmu->s2crs[idx].cbndx);
	arm_smmu_write_sme(smmu, idx);

	mutex_unlock(&smrs[idx].smr_mutex);

	return 0;
}



/* Setup of domain context by initializing cfg, cb and pgtbl_cfg for a given pasid
 * @paramm smmu_domain pointer to specific domain which will be initialized
 * @param smmu pointer to smmu device
 * @param cbndx context bank index which will be assigned to smmu_domain
 * @param pasid which will be assigned to smmu_domain
 * @return 0 on success, < 0 on failure
 */
static int arm_smmu_init_domain_context(struct arm_smmu_domain *smmu_domain, 
					struct arm_smmu_device *smmu, u8 cbndx, u32 pasid)
{
	int irq, ret = 0;
	struct io_pgtable_cfg *pgtbl_cfg;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;

	mutex_lock(&smmu_domain->domain_mutex);
	if (smmu_domain->smmu && smmu_domain->pgtbl_cfg)
		goto out_unlock;

	if (!(smmu->features & ARM_SMMU_FEAT_TRANS_S1))
		smmu_domain->stage = ARM_SMMU_DOMAIN_S2;
	if (!(smmu->features & ARM_SMMU_FEAT_TRANS_S2))
		smmu_domain->stage = ARM_SMMU_DOMAIN_S1;
	if (smmu_domain->stage != ARM_SMMU_DOMAIN_S1) {
		ret = -EINVAL;
		dev_err(smmu->dev, "Stage 1 translation is not configured\n");
		goto out_unlock;
	}
	
	/* setup cfg */
	cfg->fmt = ARM_SMMU_CTX_FMT_AARCH64;
	cfg->cbar = CBAR_TYPE_S1_TRANS_S2_BYPASS;
	cfg->cbndx = cbndx;
	cfg->task = current;
	cfg->mm = current->mm;
	cfg->irptndx = cfg->cbndx;
	cfg->asid = cfg->cbndx + smmu->cavium_id_base;
	PDEBUG("%s, cbndx: %d\n", __func__, cfg->cbndx);

	/* register memory notifier ops with tlb invalid callbacks*/
	smmu_domain->mn.ops = &mn_ops; 
	ret = mmu_notifier_register(&smmu_domain->mn, current->mm);
	if (ret)
		goto out_clear_smmu;

	/* setup page table config */
	pgtbl_cfg = kmalloc(sizeof(*pgtbl_cfg), GFP_KERNEL);
	if (!pgtbl_cfg) {
		ret = -ENOMEM;
		dev_err(smmu->dev, "Allocation for page table config failed\n");
		goto out_clear_smmu;
	}
	*pgtbl_cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= smmu->pgsize_bitmap,
		.ias		= smmu->va_size,
		.oas		= smmu->ipa_size,
		.tlb		= NULL,
		.iommu_dev	= smmu->dev,
	};
	if (smmu->features & ARM_SMMU_FEAT_COHERENT_WALK)
		pgtbl_cfg->quirks = IO_PGTABLE_QUIRK_NO_DMA;
	arm_64_lpae_pgtbl_cfg(pgtbl_cfg);


	smmu_domain->smmu = smmu;
	
	/* Initialise the context bank with our page table cfg */
	arm_smmu_init_context_bank(smmu_domain, pgtbl_cfg);
	arm_smmu_write_context_bank(smmu, cfg->cbndx);

	/*
	 * Request context fault interrupt. Do this last to avoid the
	 * handler seeing a half-initialised domain state.
	 * Threaded so that do_page_fault can sleep in handler
	 * IRQF_ONESHOT needed because interrupt line high until bottom half
	 * (context_fault()) finishes -> causes too many unhandable interrupts
	 */
	irq = smmu->irqs[smmu->num_global_irqs + cfg->irptndx];
	PDEBUG("%s, request context: irq: %d, domain cbndx: %d, domain: %llx\n", 
			__func__, irq, smmu_domain->cfg.cbndx, (u64)smmu_domain);
	ret = devm_request_threaded_irq(smmu->dev, irq, handler, arm_smmu_context_fault,
			       IRQF_ONESHOT|IRQF_SHARED, "arm-smmu-context-fault", smmu_domain);
	if (ret < 0) {
		dev_err(smmu->dev, "failed to request context IRQ %d (%u)\n",
			cfg->irptndx, irq);
		cfg->irptndx = INVALID_IRPTNDX;
	}
	
	smmu_domain->pasid = pasid;

	/* Publish page table cfg */
	smmu_domain->pgtbl_cfg = pgtbl_cfg;

	mutex_unlock(&smmu_domain->domain_mutex);

	return 0;

out_clear_smmu:
	smmu_domain->smmu = NULL;
out_unlock:
	mutex_unlock(&smmu_domain->domain_mutex);
	return ret;
}

/* Deregisters a SID range for a given base
 * @param base of SID range
 */
void arm_smmu_free_streams(u16 base) {

	struct sid_range *sids;

	/* Find existing SID range */
	list_for_each_entry(sids, &smmu_device->sid_ranges_head, list) {
		if (sids->base == base)
			goto del;
	}
	return;
del:	
	atomic_add(sids->num_sids, &smmu_device->num_sids_left);
	list_del_rcu(&sids->list);
	synchronize_rcu();
	kfree(sids);
}
EXPORT_SYMBOL_GPL(arm_smmu_free_streams);


/* Registers a range of SIDs starting at base, ending at base + num_sids - 1
 * @param base of SID range 
 * @param num_sids number of consecutive AXI IDs added to base
 * @return number of open stream match table entries on success, 
 * @return < 0 on failure
 */
int arm_smmu_init_streams(u16 base, u16 num_sids) 
{
	struct sid_range *sids;

	if (!smmu_device) {
		pr_err("SMMU device not yet initalized\n");
		return -ENXIO;
	}

	/* Count down from max numbers of possible num_sids, if result becomes
	 * negative, then not enough stream mapping table entires are available
	 * -> abort
	 */
	if (atomic_add_negative(-((int)num_sids), &smmu_device->num_sids_left)) {
		atomic_add((int)num_sids, &smmu_device->num_sids_left);
		return -ENOSR;
	}
	
	/* Check if same SID was registered before */
	rcu_read_lock();
	list_for_each_entry_rcu(sids, &smmu_device->sid_ranges_head, list) {
		if ((base >= sids->base && base < sids->base + sids->num_sids) || 
		    (base + num_sids > sids->base && base + num_sids < sids->base + sids->num_sids) ||
		    (base < sids->base && base + num_sids >= sids->base + sids->num_sids))
		{
			rcu_read_unlock();
			dev_err(smmu_device->dev, "Given SID range overlaps with existing one\n");
			return -EINVAL;
		}
	}
	rcu_read_unlock();
	
	/* New unique SID range -> add it to our SMMU sids list */
	sids = kmalloc(sizeof(*sids), GFP_KERNEL);
	if (!sids) {
		dev_err(smmu_device->dev, "Allocation for SIDs failed\n");
		return -ENOMEM;
	}
	sids->base = base;
	sids->num_sids = num_sids;
	list_add_rcu(&sids->list, &smmu_device->sid_ranges_head);

	PDEBUG("%s, done!\n", __func__);

	return atomic_read(&smmu_device->num_sids_left);
}
EXPORT_SYMBOL_GPL(arm_smmu_init_streams);


static int find_free_cbndx(struct arm_smmu_device *smmu_device) {
	int i;
	struct arm_smmu_cb *cb = smmu_device->cbs;

	for (i = 0; i < smmu_device->num_context_banks; ++i) {
		mutex_lock(&cb->cb_mutex);
		if (!cb->used) {
			cb->used = true;
			goto out;
		}
		mutex_unlock(&cb->cb_mutex);
		cb++;
	}
	return -EFAULT;
out:
	mutex_unlock(&cb->cb_mutex);
	return i;

}

/* Binds a pasid to a free cb (and therefore a domain)
 * and initializes domain
 * @param pasid
 * @return 0 on success, < 0 on failure
 */ 
int arm_smmu_bind_pasid(u32 pasid) {

	struct arm_smmu_domain *domains;
	int ret;

	PDEBUG("%s, pasid: %d\n", __func__, pasid);

	domains = smmu_device->domains;
	if (!domains) {
		dev_err(smmu_device->dev, "Domains don't exist\n");
		return -EFAULT;
	}
	
	ret = find_free_cbndx(smmu_device);
	if (ret < 0) {
		dev_err(smmu_device->dev, "No context bank available\n");
		return ret;
	}
	
	PDEBUG("%s, cbndx: %d\n", __func__, ret);
	
	ret = arm_smmu_init_domain_context(domains+ret, smmu_device, ret, pasid);
	if (ret) {
		dev_err(smmu_device->dev, "Init domain context failed\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(arm_smmu_bind_pasid);

/* Removes cb, cfg and domain for a given pasid.
 * Also takes care of removing left over SIDs if not done 
 * before with remove_streams().o
 * @param pasid
 */
void arm_smmu_unbind_pasid(u32 pasid) {

	int i, irq;
	struct arm_smmu_domain *domain = smmu_device->domains;
	struct arm_smmu_cb *cb;
	u8 cbndx, domain_offset = 0;

	PDEBUG("%s, pasid: %d\n", __func__, pasid);

	for (i = 0; i < smmu_device->num_context_banks; ++i) {
		PDEBUG("%s, domain: 0x%llx\n", __func__, (u64)domain);
		mutex_lock(&domain->domain_mutex);
		if (domain->pasid == pasid) {
			goto unbind;	
		}
		mutex_unlock(&domain->domain_mutex);
		domain++;
		domain_offset++;
	}
	dev_err(smmu_device->dev, "unbind_passid failed\n");
	return;
	
unbind:
	/* If there are still SIDs registered with a pasid remove them first */
	if (!list_empty(&domain->idxs) && domain->num_idxs) {
		PDEBUG("%s, remove all streams\n", __func__);
		/* misuse num_sids parameter and pass domain_offset instead
		 * avoids another mutex lock and for loop in remove_streams()
		 */
		arm_smmu_remove_streams(pasid, NULL, domain_offset);
	}

	/* inverse operations of init_domain_context() */
	domain->pasid = INVALID_PASID;
	cbndx = domain->cfg.cbndx;
	if (domain->cfg.irptndx != INVALID_IRPTNDX) {
		irq = smmu_device->irqs[smmu_device->num_global_irqs + domain->cfg.irptndx];
		PDEBUG("%s, free irq: %d of domain: 0x%llx\n", __func__, irq, (u64)domain);
		devm_free_irq(smmu_device->dev, irq, domain);
	}
	domain->cfg.cbndx = 0;
	/* remove memory notifier*/
	PDEBUG("%s, mn: 0x%llx, mm: 0x%llx, cfg->mm: 0x%llx\n", 
			__func__, (u64)&domain->mn, (u64)domain->cfg.task->mm, (u64)domain->cfg.mm);
	mmu_notifier_unregister(&domain->mn, domain->cfg.mm);
	domain->cfg.task = NULL;
	domain->cfg.mm = NULL;
	kfree(domain->pgtbl_cfg);	
	domain->pgtbl_cfg = NULL;
	mutex_unlock(&domain->domain_mutex);

	/* destroy context bank and invalidate tlb
	 * so that no other stream can access translation
	 * tables in TLB of same context bank
	 */
	cb = &smmu_device->cbs[cbndx];
	mutex_lock(&cb->cb_mutex);
	cb->used = false;
	cb->mair[0] = 0, cb->mair[1] = 0;
	cb->tcr[0] = 0, cb->tcr[1] = 0;
	cb->ttbr[0] = 0, cb->ttbr[1] = 0;
	cb->cfg = NULL;
	arm_smmu_write_context_bank(smmu_device, cbndx);
	arm_smmu_tlb_inv_all_s1(smmu_device, cbndx);
	mutex_unlock(&cb->cb_mutex);


	PDEBUG("%s, done\n",__func__);
	return;
}
EXPORT_SYMBOL_GPL(arm_smmu_unbind_pasid);

static bool contains_sid(u16 *sids, u16 sid, u16 num_sids) {
	u16 i;

	for (i = 0; i < num_sids; ++i)
		if (sids[i] == sid)
			return true;
	
	return false;
}

/* Removes num_sids SIDs from pasid
 * Sets s2cr and smr accordingly to init values
 * If called from unbind() it removes all SIDs
 * @param pasid
 * @param sids list of SIDs to be removed
 * @param num_sids number of SIDs in sids
 */
void arm_smmu_remove_streams(u32 pasid, u16 *sids, u16 num_sids) {

	struct arm_smmu_domain *domain = smmu_device->domains;
	struct idx *idx_info, *idx_tmp;
	int sum = 0, i;
	u8 idx;
	u16 sid;

	PDEBUG("%s, pasid: %d, num_sids: %d\n", __func__, pasid, num_sids);
	
	/* sids empty if called from smmu_unbind -> domain mutex already locked */
	if (sids) {
		for (i = 0; i < smmu_device->num_context_banks; ++i) {
			mutex_lock(&domain->domain_mutex);
			if (domain->pasid == pasid) {
				goto remove;	
			}
			mutex_unlock(&domain->domain_mutex);
			domain++;
		}
		dev_err(smmu_device->dev, "remove_streams failed");
		return;
	} else {
		/* called from unbind_pasid() -> num_sids is 
		 * now offset of domain pointer
		 */
		domain += num_sids;
		num_sids = 0;
	}
	PDEBUG("%s, domain: 0x%llx\n", __func__, (u64)domain);
	
remove:
	 list_for_each_entry_safe(idx_info, idx_tmp, &domain->idxs, list) {
		idx = idx_info->idx;
		sid = idx_info->sid;
		PDEBUG("%s, pasid: %d, idx: %d, sid: 0x%x", __func__, pasid, idx, sid);
		/* if called from unbind() (sids == NULL), remove all streams of pasid */
		if (sids == NULL || contains_sid(sids, sid, num_sids)) {
			PDEBUG("%s, pasid: %d, removing sid: 0x%x", __func__, pasid, sid);
			sum++;
			mutex_lock(&smmu_device->smrs[idx].smr_mutex);
			smmu_device->s2crs[idx].count -= 1;
			smmu_device->s2crs[idx] = s2cr_init_val;
			smmu_device->smrs[idx].valid = false;
			smmu_device->smrs[idx].sid = 0;
			arm_smmu_write_sme(smmu_device, idx);
			mutex_unlock(&smmu_device->smrs[idx].smr_mutex);
			list_del(&idx_info->list);
			kfree(idx_info);
		}
	}
	PDEBUG("%s, sum: %d, domain->num_idxs: %d\n", __func__, sum, domain->num_idxs);

	if (sum != domain->num_idxs) {
		PDEBUG("%s, number of removed idxs doesn't match added streams", __func__);
	}

	domain->num_idxs = domain->num_idxs - sum;
	
	if (sids)
		mutex_unlock(&domain->domain_mutex);
	
	PDEBUG("%s, done\n", __func__);

	return;
}
EXPORT_SYMBOL_GPL(arm_smmu_remove_streams);


static bool sid_registered(struct arm_smmu_device *smmu, u16 sid) {
	
	struct sid_range *range;

	rcu_read_lock();
	list_for_each_entry_rcu(range, &smmu->sid_ranges_head, list) {
		PDEBUG("%s, sid: 0x%x, base: 0x%x, num_sids: %d\n", 
				__func__, sid, range->base, range->num_sids);
		if (sid >= range->base && sid < range->base + range->num_sids)
			return true;
	}
	rcu_read_unlock();
	return false;
}

/* For a given (previously registered) pasid add num_sids SIDs
 * Adds each SID in a new stream match table entry
 * @param pasid
 * @param sids list of SIDs
 * @param num_sids number of SIDs in sids
 * @return 0 on success, < 0 on failure
 */
int arm_smmu_add_streams(u32 pasid, u16 *sids, u16 num_sids)
{
	int ret, i;
	u8 idx;
	u16 mask, sid;
	struct arm_smmu_domain *domain = smmu_device->domains;
	struct idx *idx_info;

	PDEBUG("%s, pasid: %d, num_sids: %d\n", __func__, pasid, num_sids);

	if (!domain) {
		dev_err(smmu_device->dev, "Domains don't exist\n");
		return -EFAULT;
	}

	/* check if pasid has been registered yet */
	for (i = 0; i < smmu_device->num_context_banks; ++i) {
		mutex_lock(&domain->domain_mutex);
		if (domain->pasid == pasid) {
			goto add;	
		}
		mutex_unlock(&domain->domain_mutex);
		domain++;
	}
	return -EINVAL;
	
add:
	/* each SID gets its own stream mapping table entry */
	domain->num_idxs += num_sids;
	
	for (i = 0; i < num_sids; ++i) {
		sid = sids[i];
		PDEBUG("%s, pasid: %d, num_sids: %d, sid: 0x%x\n", __func__, pasid, num_sids, sid);

		if (sid & ~smmu_device->streamid_mask) {
			dev_err(smmu_device->dev, "stream id  0x%x out of range for SMMU (0x%x)\n",
				sid, smmu_device->streamid_mask);
			return -EINVAL;
		}

		mask = sid >> SMR_MASK_SHIFT;
		if (mask & ~smmu_device->smr_mask_mask) {
			dev_err(smmu_device->dev, "SMR mask 0x%x out of range for SMMU (0x%x)\n",
				mask, smmu_device->smr_mask_mask);
			return -EINVAL;
		}

		if (!sid_registered(smmu_device, sid)) {
			dev_err(smmu_device->dev, "SID 0x%x is unknown. It has not been registered yet\n", sid);
			return -EINVAL;
		}

		ret = arm_smmu_init_stream_match(smmu_device, &idx, sid);
		if (ret) {
			dev_err(smmu_device->dev, "Init stream match failed\n");
			return ret;
		}
		PDEBUG("%s, idx: %d\n", __func__, idx);

		idx_info = kmalloc(sizeof(*idx_info), GFP_KERNEL);
		if (!idx_info) {
			dev_err(smmu_device->dev, "Kmalloc for idx_info failed\n");
			return -ENOMEM;
		}

		idx_info->sid = sid;
		idx_info->idx = idx;
		list_add(&idx_info->list, &domain->idxs);

		ret = arm_smmu_init_stream_context(domain, idx);
		if (ret) {
			dev_err(smmu_device->dev, "Init stream context failed\n");
			return ret;
		}
		
		ret = arm_smmu_master_write_sme(smmu_device, idx);
		if (ret) {
			dev_err(smmu_device->dev, "Master write sme failed\n");
			return ret;
		}
	}

	mutex_unlock(&domain->domain_mutex);

	return 0;	
}
EXPORT_SYMBOL_GPL(arm_smmu_add_streams);


static int arm_smmu_device_probe(struct platform_device *pdev)
{
	struct resource *res;
	resource_size_t ioaddr;
	struct arm_smmu_device *smmu;
	struct device *dev = &pdev->dev;
	int num_irqs, i, err;

	PDEBUG("%s\n", __func__);

	smmu = devm_kzalloc(dev, sizeof(*smmu), GFP_KERNEL);
	if (!smmu) {
		dev_err(dev, "failed to allocate arm_smmu_device\n");
		return -ENOMEM;
	}
	smmu->dev = dev;

	err = arm_smmu_device_dt_probe(pdev, smmu);

	if (err)
		return err;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ioaddr = res->start;
	smmu->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(smmu->base))
		return PTR_ERR(smmu->base);
	smmu->cb_base = smmu->base + resource_size(res) / 2;

	num_irqs = 0;
	while ((res = platform_get_resource(pdev, IORESOURCE_IRQ, num_irqs))) {
		num_irqs++;
		if (num_irqs > smmu->num_global_irqs)
			smmu->num_context_irqs++;
	}

	if (!smmu->num_context_irqs) {
		dev_err(dev, "found %d interrupts but expected at least %d\n",
			num_irqs, smmu->num_global_irqs + 1);
		return -ENODEV;
	}

	smmu->irqs = devm_kzalloc(dev, sizeof(*smmu->irqs) * num_irqs,
				  GFP_KERNEL);
	if (!smmu->irqs) {
		dev_err(dev, "failed to allocate %d irqs\n", num_irqs);
		return -ENOMEM;
	}

	for (i = 0; i < num_irqs; ++i) {
		int irq = platform_get_irq(pdev, i);

		if (irq < 0) {
			dev_err(dev, "failed to get irq index %d\n", i);
			return -ENODEV;
		}
		smmu->irqs[i] = irq;
		PDEBUG("%s, irq: %d added\n", __func__, irq);
	}

	err = arm_smmu_device_cfg_probe(smmu);
	if (err)
		return err;

	if (smmu->version == ARM_SMMU_V2 &&
	    smmu->num_context_banks != smmu->num_context_irqs) {
		dev_err(dev,
			"found only %d context interrupt(s) but %d required\n",
			smmu->num_context_irqs, smmu->num_context_banks);
		return -ENODEV;
	}
	
	PDEBUG("%s, num_context_irqs: %d, num_global_irqs: %d, num_irqs: %d\n",
			__func__, smmu->num_context_irqs, smmu->num_global_irqs, num_irqs);

	for (i = 0; i < smmu->num_global_irqs; ++i) {
		PDEBUG("%s, global: irq: %d\n", __func__, smmu->irqs[i]);
		err = devm_request_irq(smmu->dev, smmu->irqs[i],
				       arm_smmu_global_fault,
				       IRQF_ONESHOT | IRQF_SHARED,
				       "arm-smmu global fault",
				       smmu);
		if (err) {
			dev_err(dev, "failed to request global IRQ %d (%u)\n",
				i, smmu->irqs[i]);
			return err;
		}
	}
	

	dev_set_drvdata(dev, smmu);
	smmu_device = smmu;

	PDEBUG("%s, device: %llx, smmu: %llx\n", __func__, (u64)dev, (u64)smmu);

	platform_set_drvdata(pdev, smmu);
	arm_smmu_device_reset(smmu);
	arm_smmu_test_smr_masks(smmu);

	return 0;
}


static int arm_smmu_device_remove(struct platform_device *pdev)
{
	struct arm_smmu_device *smmu = platform_get_drvdata(pdev);

	if (!smmu)
		return -ENODEV;

	if (!bitmap_empty(smmu->context_map, ARM_SMMU_MAX_CBS))
		dev_err(&pdev->dev, "removing device with active domains!\n");

	/* Turn the thing off */
	writel(sCR0_CLIENTPD, ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sCR0);
	return 0;
}

static void arm_smmu_device_shutdown(struct platform_device *pdev)
{
	arm_smmu_device_remove(pdev);
}

static int __maybe_unused arm_smmu_pm_resume(struct device *dev)
{
	struct arm_smmu_device *smmu = dev_get_drvdata(dev);

	arm_smmu_device_reset(smmu);
	return 0;
}

static SIMPLE_DEV_PM_OPS(arm_smmu_pm_ops, NULL, arm_smmu_pm_resume);

static struct platform_driver arm_smmu_driver = {
	.driver	= {
		.name		= "hsa-smmu",
		.of_match_table	= of_match_ptr(arm_smmu_of_match),
		.pm		= &arm_smmu_pm_ops,
	},
	.probe	= arm_smmu_device_probe,
	.remove	= arm_smmu_device_remove,
	.shutdown = arm_smmu_device_shutdown,
};
module_platform_driver(arm_smmu_driver);


MODULE_DESCRIPTION("HSA SVM SMMU API");
MODULE_AUTHOR("Lukas Liebischer");
MODULE_LICENSE("GPL v2");
