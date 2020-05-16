#ifndef _HSA_SMMU_H
#define _HSA_SMMU_H

#include <linux/types.h>
#include <linux/sched.h>

extern int arm_smmu_init_streams(u16 base, u16 num_ids);
extern void arm_smmu_free_streams(u16 base);
extern int arm_smmu_add_streams(u32 pasid, u16 *sids, u16 num_sids);
extern void arm_smmu_remove_streams(u32 pasid, u16 *sids, u16 num_sids);
extern int arm_smmu_bind_pasid(u32 pasid);
extern void arm_smmu_unbind_pasid(u32 pasid);

#endif //_HSA_SMMU_H
