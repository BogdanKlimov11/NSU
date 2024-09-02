#!/usr/bin/env bash
find $1 -type f,d | sed 's/.*\///' | while read c1; do echo "${#c1}/$c1" ; done | sort -n -r | head -1 | sed 's/.*\///'
