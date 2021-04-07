#!/bin/sh
cd users
a=1

while [ $a -lt 26 ]
do
  mkdir user$a
  cd user$a
  mkdir task1
  mkdir task2
  mkdir task3
  cd ../
  a=`expr $a + 1`
done
