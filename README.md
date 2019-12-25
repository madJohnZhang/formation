# formation
### TODO
1. design the initial logic, because the initial position estimate need the concise yaw of each drone. So, I need to control the yaw, before do the pos estimate initialization. The solution is, control the yaw first based on the pos of the target in the figure and estimate after some time. And control the yaw while do estimate.

2. complete the log output.

3. test the pos est code.


#### 2019.11.29
1. set a synch memory in the posvel communication pack
2. class communication recieve the ready state from others and get informed if the node itself is ready by main function.
3. class formation in main function get ready states and self state to know whether all the agents are ready. it can be done in formation.init().
4. the communication bewteen two processes is: Control signal from main to commu; States from commu to main; Self ready state from main to commu.


#### 2019.12.5
There are several ways to realize the small parts of the system.
1. change the synchronize flag transmission type. merge the formation number and the synchronize signal into one int bytes to save communication cost.
2. the initialization part: use the static image when bounding the box; add the people detection module.
3. change the script, write the console output to a log file.
4. link the device to the specific serial name.

#### 2019.12.15
use (fuser) to check the pid pocessing the specific device

#### 2019.12.25
1. change the self and xb data output to the same file
2. add integration of PID
3. increase the formation size
4. add bound in the control queue