#!bin/bash
conda activate $0 
rosrun rosserial_python serial_node.py _port:=$1 _baud:=$2