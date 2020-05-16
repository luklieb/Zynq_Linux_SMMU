#!/bin/bash

results=./results.txt

sleep 5

make clean
make all

#types: 0=normal, 1=odp, 2=phys, 3=split, 4=repeat, 5=once_odp,
# 6=once_norm, 7=once_rep, 8=stride, 9=stride_odp, 10=stride_rep, 11=baremetal

echo "type, num_proc, num_byte, num_runs, time" >> $results

#phys addr, odp, split
for type in 2 0 1 3; do
	echo "type: $type"
	echo
	for num_proc in `seq 1 16`; do
		echo "# num_concurrent $num_conc"
		
		for num_pages in 1 2 3 4 6 8 10 12 16 24 32 64 128 256 512 1024 2048 4096 8192; do
			echo $num_pages
			./size $num_proc $num_pages $type 15 >> $results
		done
		echo
	done
	echo
done

echo "type: 5"
for num_proc in `seq 1 16`; do
	./size $num_proc 0 5 15 >> $results
done

echo "type: 6"
for num_proc in `seq 1 16`; do
	./size $num_proc 0 6 15 >> $results
done

echo "type: 8"
for num_proc in `seq 1 16`; do
	./size $num_proc 8192 8 15 >> $results
done

echo "type: 9"
for num_proc in `seq 1 16`; do
	./size $num_proc 8192 9 15 >> $results
done

make clean
make all REPT=1

echo "type: 4"
for num_proc in `seq 1 16`; do
	for num_runs in 1 2 3 4 6 8 12 16; do
		./size $num_proc 8192 4 $num_runs >> $results
	done
done

echo "type: 7"
for num_runs in 1 2 3 4 6 8 12 16; do
	./size 1 0 7 $num_runs >> $results
done

echo "type: 10"
for num_proc in `seq 1 16`; do
	for num_runs in 1 2 3 4 6 8 12 16; do
		./size $num_proc 8192 10 $num_runs >> $results
	done
done

