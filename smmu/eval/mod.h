#ifndef _HSA_MOD_H
#define _HSA_MOD_H

#include <linux/kernel.h>
#include <linux/ioctl.h>

#define SMMU_TYPE (';')
#define SMMU_BASE 0

typedef __u8 uchar;
typedef __u32 uint;
typedef __u32 u32;
typedef __u64 u64;

struct hsa_dma{
	unsigned long long int iova;
	unsigned long vaddr;
	size_t size;
};


#define HSA_PA _IOWR(SMMU_BASE, SMMU_TYPE + 0, u64)
#define HSA_FREE _IOWR(SMMU_BASE, SMMU_TYPE + 1, u64)
#define HSA_BIND _IOWR(SMMU_BASE, SMMU_TYPE + 2, u64)
#define HSA_BENCH _IOWR(SMMU_BASE, SMMU_TYPE +3, u64)

#endif //_HSA_MOD_H
