#!/bin/bash

if  [[ $# =~ 3 ]] ; then
   echo "error: args aren't valid" >&2;
   exit 1;
fi
re='^[0-9]+$'
if ! [[ $1 =~ $re ]] ; then
   echo "error: Invalid args" >&2;
   exit 1;
fi
if ! [[ $2 =~ $re ]] ; then
   echo "error: Invalid args" >&2;
   exit 1;
fi

add=$(($1 + $2)) 

echo $add


