#!/bin/bash
source /home/delta/lawnbot/venv/bin/activate
cd ~/lawnbot/robot_control
python manage.py runserver 0.0.0.0:8000

