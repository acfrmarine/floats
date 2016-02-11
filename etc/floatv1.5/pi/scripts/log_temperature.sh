#!/bin/bash


while true; do
	echo $( date ),$(vcgencmd measure_temp) >> /home/pi/temperature_log.txt
	sleep 1m
done
