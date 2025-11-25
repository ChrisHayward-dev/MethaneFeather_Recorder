#!/bin/bash
infile=$1.bin
outfile=$1.uf2
echo "Converting $infile to $outfile"
python uf2conv.py -c -f 0x68ed2b88 -b 0x2000 $infile -o $outfile
