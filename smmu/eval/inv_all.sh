#!/bin/bash

#make sure in fault.c transfers are from array a to b and not b to b

for num_pages in 1 2 4 8 16 32 64 128 256 512 1024 2048 4096 8192; do
	echo $num_pages > /dev/kmsg
	for i in $(seq 1 15); do
		./fault $num_pages 1
	done
done
