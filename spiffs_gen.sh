#!/bin/sh

mkdir -p spiffs_dir
dd if=/dev/urandom of=spiffs_dir/image.bin bs=1 count=$((0x10000))
python ../components/spiffs/spiffsgen.py 0x20000 spiffs_dir spiffs_image

../components/partition_table/parttool.py --port /dev/ttyUSB2 --partition-table-file partitions.csv write_partition --partition-name storage --input spiffs_image
