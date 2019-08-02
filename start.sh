#!bin/bash
python ./control.py &
bash -i ./rosserial_starter.sh &
echo aki
bash -i ./start_controlador.sh &
echo here
conda activate humanoid
python ./communication.py
killall -s SIGINT roscore
killall -s SIGINT rosmaster