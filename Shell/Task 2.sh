#!/usr/bin/env bash
cat /etc/passwd | cut -d ":" -f 1 | while read user
do
	proc=$(sed '2!d' <( ps -u $user ))
	if [ ! -z "$proc" ] ; then
		log=$(sed '1!d' <( last $user ))
		if [ -z "$log" ] ; then
			echo $user
		fi
	fi
done
