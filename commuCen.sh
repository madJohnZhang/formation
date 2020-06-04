#!/usr/bin/expect 
spawn sudo  su
expect "password"
sleep 0.2
send "ubuntu\r"
sleep 0.2
send "source devel/setup.bash\r"
send "rosrun uav_tracking communication cenddd 333\r"
interact