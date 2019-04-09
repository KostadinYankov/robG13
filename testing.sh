#!/bin/bash

for i in {1..5} 
do
	touch testLogRun1_$i.txt
	echo $i
	python robG13-master/ifMore1.py >> testLogRun1_$i.txt
done
