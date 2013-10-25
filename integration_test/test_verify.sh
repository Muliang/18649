#!/bin/bash
set -e
set -u
summaryfile=$1

if [[ -f "$summaryfile" ]]; then
  echo "Test summary file $summaryfile exists."
  echo
else
  echo "Test summary file $summaryfile could not be found."
  echo "Verification failed."
  exit -1
fi

cat $summaryfile | dos2unix | grep -v -e "^;" | while read controller cf mf; do
  if [[ "$controller" == "" && "$cf" == "" && "$mf" == "" ]]; then
    #echo "Skipping blank line"
    continue
  fi
  echo "Controller:  $controller  Config:  $cf  MessageInjector: $mf"
  if [[ -f "$cf" ]]; then
    echo "   $cf exists."
  else
    echo "   $cf does not exist."
    echo "Verification failed."
    exit -1
  fi
  if [[ -f "$mf" ]]; then
    echo "   $mf exists."
  else
    echo "   $mf does not exist."
    echo "Verification failed."
    exit -1
  fi      
done
if [[ "$?" -eq "0" ]]; then
  echo "Verification passed"
fi