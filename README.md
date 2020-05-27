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

#### 2019.12.26

add an information.
control signal, body frame, compared to the head direction
x->positive (forward), negative (backward)
y->positive (right), negative (left)
z->positive (up), negative (down)

#### 2020.01.14

1. filter the position. reduce the iteration times of the position estimate (not the initial part), to decrease the influence of the noise. maybe using variance reduction techniches is also a way.
2. ki for horizontal control. kp for yaw control.
3. record info from ui, filter ui to position.
4. adjust pid parameter of yaw control separately.
5. test height control.

#### 2020.01.29

1. add the decentralized version of the pos estimation.
2. think about the way of info exchange.

#### 2020.02.21

1. add the final signal output relation between ground frame and the body frame
2. ground and body frame explanation. about x and y

#### 2020.03.18

1. add the class of decentralized position esimation. extract the position estimation class as the parent class with the centralized and the decentralized inheriting publicly. 

2. 

#### 2020.05.11

Plan for experiment

1. buy two carts and backup XBees, install equipment of the third drone.
2. fix the control para of yaw, add the height control.
3. add the discrete integral of the horizontal control under formation of two agents(one in the air, one on the ground), since for now the drone is controlled by
velocity while the control logic is accelaration.
4. test the formation of three UAVs under the centralized setting.
5. the decentralized setting.
6. obstacle avoidance.

#### 2020.05.15

notes: create computation network.

#### 2020.05.26

Dilemma: dec pos est seems not to work, which is the value cannot converge or converge into a wrong position. My diagnose is that two values from two nodes are not in the same iteration, maybe a static gap, after lots of atempts. There are two assumed reasons: communication; the main process runs faster than the communication process and the same value can be calculated mltiple times.

Solution: add a synchronized mechanism. Producer-consumer model. 0-1 good, produce queue (produce flag merged in number and synch signal), since one communication loop means a value comming (new or old). Design carefully in case of more bugs.
