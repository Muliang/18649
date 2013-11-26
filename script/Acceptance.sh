#!/bin/bash

rm -f *.stats


set -u
set -e


ls acceptance_test/ | awk '/pass/' | while read acceptance_test; do
  if [[ "$acceptance_test" == "" ]]; then
    #echo "Skipping blank line"
    continue
  fi
  if [[ -f "acceptance_test/$acceptance_test" ]]; then
    
    java simulator/framework/Elevator -b 200 -fs 5.0 -head HEADER -pf acceptance_test/$acceptance_test -monitor Proj11RuntimeMonitor | grep "Acceptance\|RandomSeed\|Delivered\|Stranded\|Total\|simulation seconds\|performance\Monitors Summmary Results\RT6\RT7\RT8.1\RT8.2\RT8.3\RT9\RT10"
  else
    echo "$acceptance_test does not exist."
    echo "Verification failed."
    exit -1
  fi

#grep output.txt
done
if [[ "$?" -eq "0" ]]; then
  echo "Verification passed"
fi

#rm -f *.stats
