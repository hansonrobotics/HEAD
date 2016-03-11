#!/usr/bin/env sh

rosnode kill /sophia_body/pololu_body
sleep 1
rosnode kill /sophia_body/pololu_head
sleep 1
rosnode kill /sophia_body/pololu_arm
sleep 1
rosrun dynamic_reconfigure dynparam set /sophia_body/pau2motors reload True
