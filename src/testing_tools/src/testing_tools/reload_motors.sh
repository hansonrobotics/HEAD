#!/usr/bin/env sh

rosnode kill /sophia_body/pololu_body
rosnode kill /sophia_body/pololu_head
sleep 2
rosrun dynamic_reconfigure dynparam set /sophia_body/pau2motors reload True
