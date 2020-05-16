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


int main(int argc, char **argv) {
	int dev;

	dev = open(HSA_DEV, O_RDWR);
	if (dev < 0) {
		printf("failed dev open\n");
		goto out;
	}

	for (int i = 1; i <= 16; ++i) {
		ioctl(dev, HSA_BENCH, &i);
	}
	close(dev);
out:
	return EXIT_SUCCESS;
}

