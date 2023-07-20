#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./kd_variable_rt
else
	EXEC=./kd_variable_nrt
fi

$EXEC -n x8_0 -a 127.0.0.1 -p 9000 -l ./ -x setup_x8.xml -t x8_simu
