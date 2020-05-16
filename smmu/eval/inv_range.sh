#!/bin/bash

#make sure in fault.c transfers are from array a to b and not b to b
file=results_inv_range.txt

echo "num_pages, svm, time" >> $file

for svm in 0 1; do
	for num_pages in 1 2 4 8 16 32 64 128 256 512 1024 2048 4096 8192; do
		echo ", $num_pages" > /dev/kmsg
		for i in $(seq 1 10); do
			#./inv_range $num_pages $svm >> $file
			./inv_range $num_pages $svm
		done
	done
done
