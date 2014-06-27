echo "Starting"
tmux new-session -n 'roscore' -d 'roscore; $SHELL'
sleep 3;
tmux new-window -n 'rosbridge' 'roslaunch rosbridge_server rosbridge_websocket.launch; $SHELL'
tmux new-window -n 'ros_pololu' 'rosrun ros_pololu_servo ros_pololu_servo_node; $SHELL'
tmux new-window -n 'pau2motors' 'rosrun pau2motors pau2motors_node.py; $SHELL'
tmux new-window -n 'expr_node' 'rosrun basic_head_api head_ctrl.py; $SHELL'
tmux new-window -n 'webserver' 'python -m SimpleHTTPServer; $SHELL'
tmux attach;
echo "Started"
