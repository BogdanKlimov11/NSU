#!/usr/bin/env bash

cut -f 1,6 -d: /etc/passwd | tr ":" " " | while read -r f1 f6 
do
	if [ "$f6" != "/nonexistent" ] ; then
		log=$(sed '1!d' <( last $f1 ))
                	if [ ! -z "$log" ] ; then
                        	echo $f1
                	fi
	fi
done

