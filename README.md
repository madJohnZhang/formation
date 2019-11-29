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