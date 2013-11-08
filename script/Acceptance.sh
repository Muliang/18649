#!/bin/bash

set -u
set -e


ls acceptance_test/ | awk '/pass/' | while read acceptance_test; do
  if [[ "$acceptance_test" == "" ]]; then
    #echo "Skipping blank line"
    continue
  fi
  if [[ -f "acceptance_test/$acceptance_test" ]]; then
    
    java simulator/framework/Elevator -rt 7000s -head HEADER -pf acceptance_test/$acceptance_test | grep "Acceptance\|RandomSeed\|Stranded\|Violated"
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

rm -f *.stats
