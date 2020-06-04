spawn sudo  su
expect "password"
sleep 0.2
send "ubuntu\r"
sleep 0.2
send "source devel/setup.bash\r"
send "rostopic echo posvel_msg\r"
interact