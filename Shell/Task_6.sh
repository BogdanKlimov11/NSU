#!/bin/bash

N=0;
SUM=0;
while read -a LINE; do
	SUM=$[SUM+LINE[4]]
	N=$[N+1];
done;
echo $[SUM/N]
