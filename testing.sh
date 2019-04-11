#!/bin/bash

for i in {1..2} 
do
	touch logRun6_$i.txt
	echo $i
	python ifMore1.py >> logRun6_$i.txt
done
