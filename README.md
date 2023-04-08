git repo: https://github.com/arnygaard/ENPM661-Proj3-part2.git

Adam Nygaard, anygaard, 119457894

Bob Reiter, reiter22, 119457924

Part 01:

Libraries needed to run code:

matplotlib, numpy as np, cv2 as cv, matplotlib.pyplot as plt, math, time, sys, queue import PriorityQueue

recommended starting values:
start x: 50
start y: 100
start ange: 0
goal x: 580
goal y: 190
rpm 1: 5
rpm 2: 10
clearance (mm): 200

In order to run the algorithm correctly, first make sure to run the code in a compiler. I used VSC. This has not been tested running off a terminal command.

When running the code, you will be asked to input start and goal points. Make sure to enter each number seperately, as stated by the instructions in the terminal. Only enter one coordinate at a time, as the code will not accept xy coordinate pairs for start and goal nodes.

If you input coordinates inside an obstacle, the program will output a failure message and will need to be rerun.


Part 02:
Libraries needed to run code:

matplotlib, numpy as np, cv2 as cv, matplotlib.pyplot as plt, math, time, sys, queue import PriorityQueue

To run the ros package, run the roslaunch command in the terminal along with the launch file. For us, the following line was used to launch the turtlebot into the map world:

$roslaunch turtlebot3_gazebo turtlebot3_map.launch

then, navigate to the directory where nav.py is located (src folder) and then run the following command

$python3 nav.py

then input the goal coordinates similar to as above. The recommended coordinates are:

goal x: 580
goal y: 180

the algorithm will then find the path automatically and begin publishing the corresponding velocity values to the turtlebot.
