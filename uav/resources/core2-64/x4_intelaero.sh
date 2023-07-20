#! /bin/bash

if [ -f /proc/xenomai/version ];then
	EXEC=./kd_variable_rt
else
	EXEC=./kd_variable_nrt
fi

$EXEC -n x4_0 -a 172.26.213.62 -p 9000 -l /tmp -x setup_x4_intelaero.xml -t aero
