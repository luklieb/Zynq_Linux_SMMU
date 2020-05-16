#!/bin/bash

results=./results_async.txt


echo "total_byte, bandwidth" >> $results

for num_pages in 1 2 3 4 6 8 10 12 16 24 32 64 128 256 512 1024 2048 4096 8192; do
	echo $num_pages
	./async $num_pages >> $results
done
