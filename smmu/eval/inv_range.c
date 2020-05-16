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

#define HSA_DEV		"/dev/hsa/hsa"

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
void do_dma(const u64 src_iova, const u64 dst_iova, const size_t size_to_map, const uchar *base_regs) {

	//printf("Successful MMAP of AXI DMA to address %p\n", base_regs);
	// AXI DMA transfer, with tx looped back to tx, no SG, polled I/O
	// reset both tx and rx channels and wait for the reset to clear for the last one
	//printf("src_iova: %p, dst_iova: %p, size: %lld, base_reg: %p\n", 
	//	src_iova, dst_iova, size_to_map, base_regs);


	*(u32 *)(base_regs + 0x00) = 4;
	*(u32 *)(base_regs + 0x30) = 4;

	while (	*(u32 *)(base_regs + 0x30) & 0x4);

	// Start the rx transfer
	*(u32 *)(base_regs + 0x30) = 1;
	*(u32 *)(base_regs + 0x48) = (u32) dst_iova;
	*(u32 *)(base_regs + 0x4c) = (u32) (dst_iova >> 32);
	*(u32 *)(base_regs + 0x58) = size_to_map;

	// Start the tx transfer
	*(u32 *)(base_regs) = 1;
	*(u32 *)(base_regs + 0x18) = (u32) src_iova;
	*(u32 *)(base_regs + 0x1c) = (u32) (src_iova >> 32);	
	*(u32 *)(base_regs + 0x28) = size_to_map;

	while ((*(u32 *)(base_regs + 0x34) & 0x1000) != 0x1000);

}

int main(int argc, char **argv) {
	int dev;
	int id; 
	size_t num_pages, size;
	bool perm;
	int *a, *b;
	u64 dma_addr;
	struct timespec t_s, t_e;
	bool svm;
	int ret = EXIT_SUCCESS;

	if (argc != 3) {
		printf("aborting... wrong num of args\n");
		return EXIT_FAILURE;
	}

	num_pages = atol(argv[1]);
	svm = atoi(argv[2]);

	size = num_pages * (1<<12);

	a = (int *) malloc(size);
	b = (int *) malloc(size);
	if (!a || !b) {
		printf("malloc failed\n");
		return EXIT_FAILURE;
	}

	for (int i = 0; i < size/sizeof(*a); ++i) {
		a[i] = i;
		b[i] = i + 1;
	}
	if (!svm)
		goto regular;

	dev = open(HSA_DEV, O_RDWR);
	if (dev < 0) {
		printf("failed dev open\n");
		goto out;
	}

	read(dev, &id, sizeof(id));	
	
	ioctl(dev, HSA_BIND, NULL);
	
	dma_addr = dma_addrs[id]; 

	int fd_mem = open("/dev/mem", O_RDWR | O_SYNC);
	const uchar *base_regs = (uchar *)mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, dma_addr);
	if (base_regs == MAP_FAILED) {
		printf("Failed MMAP of AXI DMA at address 0x%lx\n", dma_addr);
		close(fd_mem);
		return -ENOMEM;
	}
	
	do_dma((u64) a, (u64) b, size, base_regs);

	for (int i = 0; i < size/sizeof(*a); ++i) {
		if (a[i] != b[i] || b[i] != i) {
			ret = EXIT_FAILURE;
			printf("transaction failed\n");
			goto out;
		}	
	}

out:
	clock_gettime(CLOCK_MONOTONIC, &t_s);
	free(a);
	free(b);
	clock_gettime(CLOCK_MONOTONIC, &t_e);
	printf("%d, %d, %f\n", 
		num_pages,
		svm,
		(((double)t_e.tv_sec + t_e.tv_nsec/1.e9) - ((double)t_s.tv_sec + t_s.tv_nsec/1.e9))*1.e9/2.);
	
	close(dev);
	return ret;






regular:
	for (int i = 0; i < size/sizeof(*a); ++i) {
		b[i] = a[i];
	}
	clock_gettime(CLOCK_MONOTONIC, &t_s);
	free(a);
	free(b);
	clock_gettime(CLOCK_MONOTONIC, &t_e);
	printf("%d, %d, %f\n", 
		num_pages,
		svm,
		(((double)t_e.tv_sec + t_e.tv_nsec/1.e9) - ((double)t_s.tv_sec + t_s.tv_nsec/1.e9))*1.e9/2.);
	return ret;
}

