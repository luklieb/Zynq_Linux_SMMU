// The purpose of this test is to show that VFIO works and allows a simple user space DMA solution.

// This assumes the kernel has the VFIO platform driver built into it (as a module)
// The AXI DMA should be in the device tree with the iommu property.

// The hardware must have been built properly to allow the DMA to IOMMU (SMMU) to the DMA to work with
// virtual addresses. It assumes the AXI DMA is connected to HPC0 of the MPSOC.

// The vfio driver is used with the following commands prior to running this test application.
// Maybe it can be used from the device tree like UIO, but not sure yet.

#include <linux/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdbool.h>

#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/eventfd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <wait.h>
#include <sys/ioctl.h>
#include <math.h>

#include "mod.h"

typedef __u8 uchar;
typedef __u32 uint;
typedef __u32 u32;
typedef __u64 u64;

#define SWAP

#define HSA_DEV		"/dev/hsa/hsa"

#define BUS_WIDTH 16 //128 bit in byte

static u64 dma_addrs[] = {     
	0xa0000000,
	0xa0001000,
	0xa0002000,
	0xa0003000,
	0xa0004000,
	0xa0005000,
	0xa0006000,
	0xa0007000,
	0xa0008000,
	0xa0009000,
	0xa000A000,
	0xa000B000,
	0xa000C000,
	0xa000D000,
	0xa000E000,
	0xa000F000
};

//Types
#define NORM 0
#define ODP  1
#define PHYS 2
#define SPLT 3
#define REP  4
#define ONCE_ODP 5
#define ONCE_NORM 6
#define ONCE_REP 7
#define STRIDE 8
#define STRIDE_ODP 9
#define STRIDE_REP 10
//REPT set by compiler

//SPLT: split num_pages by num_proc, so that, no matter how many num_proc, the same amount of data is transferred
//REP: each run same memory, 
//ONCE_ODP: each run new malloc/free, odp is on -> TBU miss, pagefault, SMMU table walk
//ONCE_NORM: each run new malloc/free, odp is off -> TBU miss, SMMU table walk
//ONCE_REP: each run same memory, odp is on -> first TBU miss, pagefault, SMMU table walk, after that TBU hits (pte already in TBU)
//STRIDE: only every first element in a page is transferred, odp is off -> TBU miss, SMMU table walk
//STRIDE_ODP: odp on -> TBU miss, pagefault, SMMU table walk
//STRIDE_REP: each run same memory, odp in on (once) -> first TBU miss, pagefault, SMMU table walk, after that TBU hits (pte already in TBU)

#define MAX_SIZE_BITS	25
#define MAX_SIZE_MASK	(1<<MAX_SIZE_BITS)-1
#define NUM_BURST 16
#define ALIGNMENT 1<<12
#define LOOP_STRIDE 1<<12

static double wall_time = 0.;

struct range {
	u64 addr;
	unsigned long order;
};

void timing(double *);

void inline timing(double* wcTime)
{
   struct timeval tp; 

   gettimeofday(&tp, NULL);
   
   *wcTime=(double) (tp.tv_sec*1.e6 + tp.tv_usec); 
}

void do_dma(const u64 src_iova, const u64 dst_iova, const size_t size_to_map, 
		const uchar *base_regs, bool strided) {

	//printf("Successful MMAP of AXI DMA to address %p\n", base_regs);
	// AXI DMA transfer, with tx looped back to tx, no SG, polled I/O
	// reset both tx and rx channels and wait for the reset to clear for the last one
	//printf("src_iova: %p, dst_iova: %p, size: %lld, base_reg: %p\n", 
	//	src_iova, dst_iova, size_to_map, base_regs);
	struct timespec t_s, t_e;

	*(volatile u32 *)(base_regs + 0x00) = 4;
	*(volatile u32 *)(base_regs + 0x30) = 4;

	//printf("%s, wait for reset\n", __func__);
	while (	*(volatile u32 *)(base_regs + 0x30) & 0x4);
	//printf("%s, wait for reset done\n", __func__);

	// Start the rx transfer
	*(volatile u32 *)(base_regs + 0x30) = 1;
	*(volatile u32 *)(base_regs + 0x48) = (u32) dst_iova;
	*(volatile u32 *)(base_regs + 0x4c) = (u32) (dst_iova >> 32);
	*(volatile u32 *)(base_regs + 0x58) = size_to_map;

	// Start the tx transfer
	*(volatile u32 *)(base_regs) = 1;
	*(volatile u32 *)(base_regs + 0x18) = (u32) src_iova;
	*(volatile u32 *)(base_regs + 0x1c) = (u32) (src_iova >> 32);
	
	*(volatile u32 *)(base_regs + 0x28) = size_to_map;
	clock_gettime(CLOCK_MONOTONIC, &t_s);

	while ((*(volatile u32 *)(base_regs + 0x34) & 0x1000) != 0x1000);
	clock_gettime(CLOCK_MONOTONIC, &t_e);

	if (strided)
		wall_time += ((double)t_e.tv_sec + t_e.tv_nsec/1.e9) - ((double)t_s.tv_sec + t_s.tv_nsec/1.e9);	
	else
		wall_time = ((double)t_e.tv_sec + t_e.tv_nsec/1.e9) - ((double)t_s.tv_sec + t_s.tv_nsec/1.e9);	
}	

int start_dma (const u64 src_va, const u64 dst_va, const size_t size, 
		const uchar *base_regs, const int id, const int stride) {
	
	size_t trans;
	u64 src_va_tmp = src_va;
	u64 dst_va_tmp = dst_va;
	size_t size_tmp = size;
	int i = 0;

	if (stride == 4) {
		do_dma(src_va, dst_va, size, base_regs, false);
	} else {
		size_t trans = NUM_BURST*BUS_WIDTH;
		u64 src_va_tmp = src_va;
		u64 dst_va_tmp = dst_va;
		size_t size_tmp = size;
		while (size_tmp > 0) {
			++i;
			do_dma(src_va_tmp, dst_va_tmp, trans, base_regs, true);
			src_va_tmp += stride;
			dst_va_tmp += stride;
			size_tmp -= stride;
		}
		//printf("wall_time: %f, size: %lu, iter: %d\n", wall_time, size, i);
	}

	return 0;
}

void init_values (const u64 src_va, const u64 dst_va, const size_t size, int type) 
{
	int *src_ptr = (int *)((uintptr_t)src_va);
	int *dst_ptr = (int *)((uintptr_t)dst_va);
	bool cond = type != ODP && type != REP && type != ONCE_ODP && type != ONCE_REP && type != STRIDE_ODP && type != STRIDE_REP;	
	// fill with random data
	size_t c, tot = size/sizeof(*src_ptr);
	srand(time(NULL));
	for (c = 0; c < tot; c++) {
		src_ptr[c] = rand();
		if (cond)
			dst_ptr[c] = 1;
	}
}

void compare_values (const u64 src_va, const u64 dst_va, const size_t size, const int stride) {

	int *src_ptr = (int *)((uintptr_t)src_va);
	int *dst_ptr = (int *)((uintptr_t)dst_va);
	size_t c, tot = size/sizeof(*src_ptr);
	size_t fail_count = 0, success_count = 0;
	
	// Compare the destination to the source to make sure they match after the DMA transfer
	for(c = 0; c < tot; c+=stride/sizeof(*src_ptr)) {
		if(src_ptr[c] != dst_ptr[c]) {
			fail_count++;
		}else{
			success_count++;
		}
	}

	if (fail_count) { 
		printf("test failed %lld, succeeded %lld for proc %d\n", fail_count, success_count, getpid());
		/*for (c = 0; c < 5; ++c) {
			printf("src = 0x%x, at %d\n", ((uint *)src_ptr)[c], c);
			printf("dst = 0x%x, at %d\n", ((uint *)dst_ptr)[c], c);
			printf("src = 0x%x, at %d\n", ((uint *)src_ptr)[tot-c-1], tot-c-1);
			printf("dst = 0x%x, at %d\n", ((uint *)dst_ptr)[tot-c-1], tot-c-1);
			printf("\n");
		}*/
	}
}

int run(int argc, char **argv, pid_t ppid, int fd) {
	
	unsigned int i;
	int ret = 0, device;
	int id = 0;
	int *a;
	size_t size_to_map; 
	size_t size_trans;
	sigset_t set;
	int sig;	
	const pid_t pid = getpid();
	size_t num_pages;
	int num_proc;
	int num_runs;
	u64 dma_addr;
	int val = getpid();
	int type;
	u64 src_addr;
	u64 dst_addr;
	int stride = 4;
	
	sigemptyset(&set);
	if(sigaddset(&set, SIGUSR2) == -1) {                                           
    		perror("Sigaddset error");                                                  
    		return EXIT_FAILURE;                                                    
 	}
	sigprocmask(SIG_BLOCK, &set, NULL);
	
	num_proc = atoi(argv[1]);
	num_pages = atoll(argv[2]);
	type = atoi(argv[3]);
	num_runs = atoi(argv[4]);

	if (type == SPLT)
		num_pages /= num_proc;
	
	size_to_map = sysconf(_SC_PAGESIZE) * num_pages;
	
	if (type == ONCE_ODP || type == ONCE_NORM || type == ONCE_REP)
		size_to_map = NUM_BURST*BUS_WIDTH;

	if (type == STRIDE || type == STRIDE_ODP || type == STRIDE_REP)
		stride = LOOP_STRIDE;

	device = open(HSA_DEV, O_RDWR);
	if (device < 0) {
		perror("device open failed");
		return errno;
	}	

	// Get assigned AXI ID from kernel module
	// Needed later to start transaction on correct DMA core
	read(device, &id, sizeof(id));
	id &= 0xffff;
	if (type != PHYS)
		ioctl(device, HSA_BIND, NULL);


	dma_addr = dma_addrs[id];
	
	int fd_mem = open("/dev/mem", O_RDWR | O_SYNC);
	const uchar *base_regs = (uchar *)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, dma_addr);
	if (base_regs == MAP_FAILED) {
		printf("Failed MMAP of AXI DMA at address 0x%lx\n", dma_addr);
		close(fd_mem);
		return -ENOMEM;
	}
	
	//printf("%s, base_regs: 0x%llx\n", __func__, (u64) base_regs);
	//printf("pid %d, read done, id: %d\n", pid, id);
#ifndef REPT
	for (i = 0; i < num_runs; ++i) {
#endif	
		u64 src_va, dst_va;
		// Get a buffer for the source of the DMA transfer and then map into the IOMMU
		if (type != PHYS) 
			src_va = (u64) aligned_alloc(ALIGNMENT, size_to_map);
		else 
			src_va = (u64) mmap(NULL, size_to_map, PROT_READ | PROT_WRITE, MAP_SHARED, device, 0);
		if (!src_va) {
			perror("aligned_alloc src failed\n");
			return EXIT_FAILURE;	
		}

		// Get a buffer for the destination of the DMA transfer and then map it into the IOMMU
		if (type != PHYS)
			dst_va = (u64) aligned_alloc(ALIGNMENT, size_to_map);
		else 
			dst_va = (u64) mmap(NULL, size_to_map, PROT_READ | PROT_WRITE, MAP_SHARED, device, 0);
		if (!dst_va) {
			perror("aligned_alloc dst failed\n");
			return EXIT_FAILURE;	
		}
		//printf("src_va: %p, dst_va: %p\n", src_va, dst_va);

#ifdef REPT
	for (i = 0; i < num_runs; ++i) {
#endif	

		init_values((u64)src_va, (u64)dst_va, size_to_map, type);

		if (type == PHYS) {
			src_addr = src_va;	
			ioctl(device, HSA_PA, &src_addr);
			//printf("paddr of src_va 0x%llx is 0x%llx\n", src_va, src_addr);
			dst_addr = dst_va;	
			ioctl(device, HSA_PA, &dst_addr);
			//printf("src_pa: %p, dst_pa: %p\n", src_addr, dst_addr);
			//printf("paddr of dst_va 0x%llx is 0x%llx\n", dst_va, dst_addr);
		}
		
#ifdef SWAP
		volatile int *tmp;
		u64 size_tmp = 1.8 * (1<<30);
		if (type != PHYS)
			tmp = (volatile int *) aligned_alloc(ALIGNMENT, size_tmp);
		for (u64 i = 0; i < size_tmp/sizeof(u32); ++i) {
			tmp[i] = i+2;
		}
#endif
		write(fd, &val, sizeof(val));
		//printf("pid: %d wrote to pipe\n", pid);

		sigwait(&set, &sig);
		if (sig != SIGUSR2) {
			printf("pid: %d, signal: %d\n", pid, sig);
			close(device);
			return -EXIT_FAILURE;
		}
		sigprocmask(SIG_BLOCK, &set, NULL);
		//printf("pid: %d, starting dma now\n", pid);

		if (type != PHYS)
			start_dma(src_va, dst_va, size_to_map, base_regs, id, stride);
		else
			start_dma(src_addr, dst_addr, size_to_map, base_regs, id, stride);

		compare_values(src_va, dst_va, size_to_map, stride);
		
		size_trans = size_to_map;
		if (type == ONCE_NORM || type == ONCE_ODP || type == ONCE_REP)
			size_trans = size_to_map * stride / BUS_WIDTH;

		if (type == NORM || type == ODP || type == PHYS || type == SPLT || type == REP) {
			// MB/s
			printf("%d, %d, %d, %d, %f\n", 
				type,
				num_proc,
				size_to_map, //should be 2*size_to_map, but taken care of in plot.py
				num_runs,
				((2.0 * (double)size_trans / (stride/sizeof(int)))/1.e6)/(wall_time/(1+0)));
		} else {
			// s/transac
			printf("%d, %d, %d, %d, %f\n", 
				type,
				num_proc,
				size_to_map,
				num_runs,
				1.e9*wall_time/(size_trans/stride)); //should be 2*size_trans, but taken care of in plot.py
			wall_time = 0;
		}
#ifdef REPT
	}
#endif
		if (type != PHYS) {
			free((void*)src_va);
			free((void*)dst_va);
		} else {
			struct range range = {
				.addr = src_addr, 
				.order = log2((double)size_to_map-1.) - 12. +1.,
			};
			munmap((void*)src_va, size_to_map);
			ioctl(device, HSA_FREE, &range);
			range.addr = dst_addr;
			munmap((void*)dst_va, size_to_map);
			ioctl(device, HSA_FREE, &range);
		}

#ifdef SWAP
		free((void *)tmp);
#endif

#ifndef REPT
	}
#endif

	close(fd);
	close(device);
	
	if (munmap((void *)base_regs, 0x1000)) {
		perror("error unmapping\n");
		return -1;
	}
	
	close(fd_mem);

	return EXIT_SUCCESS;
}

int main(int argc, char **argv) {
	int num_proc, i, j;
	pid_t pid, wpid;
	const pid_t ppid = getpid();
	int stat;
	int init_count = 0;
	int type;
	size_t num_pages;
	int num_runs;

	if (argc != 5) {
		fprintf(stderr, 
			"wrong arguments. call as %s [num_proc] [num_pages] [test_type (0: regular) (1: odp) (2: phys) (3: split), ...], [num_runs]]\n", 
			argv[0]);
		return EXIT_FAILURE;
	}
	
	num_proc = atoi(argv[1]);
	num_pages = atoll(argv[2]);
	type = atoi(argv[3]);
	num_runs = atoi(argv[4]);

	pid_t pids[num_proc];
	int fd[num_proc][2];

	if (type == SPLT && num_pages < 16)
		return EXIT_SUCCESS;

	for (i = 0; i < num_proc; ++i) {
		pipe(fd[i]);
		pid = fork();
		if (pid == 0) {
			close(fd[i][0]);
			return run(argc, argv, ppid, fd[i][1]);
		} else if (pid > 0) {
			pids[i] = pid;
		} else {
			printf("pid error\n");
			return EXIT_FAILURE;
		}
	}
	//printf("done spawning children\n");

	int val;
	size_t read_b = 0;
	for (j = 0; j < num_runs; ++j) {
		//printf("num_run: %d\n", j);
		for (i = 0; i < num_proc; ++i) {
			//printf("wating for pid %d\n", pids[i]);
			while (read_b != sizeof(val)) {
				//printf("init_count: %d\n", init_count);
				read_b = read(fd[i][0], &val, sizeof(val));
				init_count++;
				//printf("read %lld bytes with value %d\n", read_b, val);
			}
			read_b = 0;
			close(fd[i][1]);
		}
	
		//printf("waiting for all children done...\n");
	
		for (i = 0; i < num_proc; ++i) {
			//printf("send SIGUSR2 to pid %d\n", pids[i]);
			kill(pids[i], SIGUSR2);
		}
	}
	
	//printf("all done!\n");
	
	while ((wpid = wait(&stat)) > 0) {
		//printf("wpid: %d, normally: %d, status: %d, signaled: %d, signalterm: %d\n",
 		//	       wpid, WIFEXITED(stat), WEXITSTATUS(stat), WIFSIGNALED(stat),
		//		WTERMSIG(stat));	       
	}	

out:
	return EXIT_SUCCESS;
}

