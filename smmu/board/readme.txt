dma_multiple.bit.bin has two DMA cores, one at address 0xa0000000
one at address 0xa0001000. The first one's AXI ID is 0, the second
one's is 1.

The Board has to be booted with DT linux.dtb. The compatible field
of the SMMU was changed to 'i3,smmu-1' in order to make use of the
newly created SMMU driver located in ../linux-kernel
