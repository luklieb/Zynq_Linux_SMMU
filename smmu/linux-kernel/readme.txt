
Most important file is hsa-smmu.c. All others were edited to
add debug information only. To remove debug info simply ignore
those files (don't copy them to the kernel source directory for 
compilation) or turn them off by undefining SMMU_DEBUG 

make get: copy the specified files from kernel source directory here
make put: other way
