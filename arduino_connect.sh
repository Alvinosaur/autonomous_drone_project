echo "USB Port: $1"
rosrun rosserial_python serial_node.py /dev/$1
