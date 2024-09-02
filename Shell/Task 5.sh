ls -lR | grep '^-' | awk '{total += $5; square += $5*$5; N += 1} END {if (N) print "SD:", sqrt(total/N + square/N/N); else print "Empty Directory"}'
