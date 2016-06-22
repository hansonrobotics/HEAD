for i in $(env|grep ROS|cut -d= -f1); do
    unset $i
done
unset HR_WORKSPACE
