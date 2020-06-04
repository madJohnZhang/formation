#!/usr/bin/expect 
spawn sudo  su
expect "password"
sleep 0.2
send "ubuntu\r"
sleep 0.2
send "source devel/setup.bash\r"
send "rosrun uav_tracking main cen 3333 > mainTra.log\r"
interact