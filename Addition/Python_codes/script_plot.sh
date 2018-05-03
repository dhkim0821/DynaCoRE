#!/bin/bash
plot_these=(plot_joint 
	plot_body 
	plot_motor_current
    plot_torque)

python plot_multiple.py  ${plot_these[*]}
