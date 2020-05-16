# Projects
1. Bram
2. DMA
3. Wifi (DT overlay to activate wifi access point)
4. GPIO (only for Ultrazed board)
5. SMMU (coming soon)

All necessary files to boot/configure each project are available in the
respective directory. Just copy and paste them either on the boot- or root-
partition of your SD-card


# Build kernel + root FS
Build kernel: 

`cd buildkernel`

`./buildkernel.sh`

Build root FS:

`cd buildrootfs`

`./build_rootfs.sh`

Both are adapted from [this repository](https://github.com/ikwzm/ZynqMP-FPGA-Linux)

# Build boot essentials

for example for project *dma*:

- boot.bin (fsbl.elf, pmufw.elf, bl31.elf, u-boot.elf, regs.init, boot.bif)
- Image
- uEnv.txt
- Devicetree binaries (linux.dtb (pcw.dtsi, system-top.dts, zynmp-clk-ccf.dtsi, zynqmp.dtsi), 
dma.dtbo (pl.dtsi))
- Bitfiles (dma.bit.bin (bit_to_bin.bif))
- Root FS

Where to place them:

- boot-partition: boot.bin, Image, uEnv.txt, linux.dtb
- root-paritition: Root FS
- /lib/firmware: dma.bit.bin, dma.dtbo

All boot essentials are already available in each project folder. 
However, if you want to build them yourself, here are some hints:

## boot.bin

### fsbl.elf + pmufw.elf

Need to be rebuilt after each change of the block design

Create block design in Vivado 2018.2; export Hardware; launch SDK

New Application Project -> choose either psu_cortexa53_0 or psu_pmu_0 -> 
choose either ZynqMP FSBL or ZynMP PMU Firmware

Important: For Ultra96 set stdin and stdout to psu_uart_1 in all Board Support Packages

### bl31.elf

Can be reused for each project

Follow steps [here](http://www.wiki.xilinx.com/Build+ARM+Trusted+Firmware+%28ATF%29)

### u-boot.elf

Can be reused for each project

Follow steps [here]( http://www.wiki.xilinx.com/Build%20U-Boot)

Target: xilinx_zynqmp_zcu100_revC_defconfig

Configuration file can be found in directory boot/u-boot-config

### boot.bif + regs.init

Use boot.bif from *dma* project as template. Make sure it includes the [init] attribute 
for the register initialization

regs.init is neccessary to set a cache related register at boot time

`source Vivado settings.sh` -> `bootgen -image boot.bif -arch zynqmp -o BOOT.bin -w`


## Image

See section about build kernel

## uEnv.txt

Template can be found in directory /boot

## Devicetree binaries

Need to be rebuilt after each change of the block design

Create block design in Vivado 2018.2; export Hardware; launch SDK

New Board Support Package Project -> OS = device_tree

Important: Xilinx SDK devicetrees are often broken. So use either a devicetree 
binary from the kernel build, or from one of the projects


## Bitfiles

Need to be rebuilt after each change of the block design

Create block design in Vivado 2018.2; export Hardware; copy *.bit file

To create a *.bin file out of the *.bit file use bit_to_bin.bif and

`source Vivado settings.sh` -> `bootgen -image bit_to_bin.bif -arch zynqmp -o ./dma.bit.bin -w`


## Root FS

see section above

# DTC
`dtc -O <output_format> -@ -b 0 -o <output_file_name> <input_file_name>`

-O: Output format, either **dtb** (binary) or **dts** (source)

-@: creates \_\_symbols\_\_ region in order to resolve phandles when overlays are used

-b: specifies CPU number

-o: Output file name


# Other

wifi-password: I<3HSA12

root-password: I<3HSA
