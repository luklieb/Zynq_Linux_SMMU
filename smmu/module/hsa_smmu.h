#ifndef _HSA_SMMU_H
#define _HSA_SMMU_H

#include <linux/types.h>
#include <linux/sched.h>

int arm_smmu_init_streams(u32);
void arm_smmu_free_streams(void);
int arm_smmu_add_stream(u16);
int arm_smmu_remove_stream(u16);


#endif //_HSA_SMMU_H
