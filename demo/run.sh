#!/bin/bash         

gepetto-gui &
hpp-rbprm-server &
python3 -i ./$1

pkill -f  'gepetto-gui'
pkill -f  'hpp-rbprm-server'
