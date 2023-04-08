import matplotlib.pyplot as plt
import numpy as np
import math
import time
from queue import PriorityQueue
import cv2 as cv
import sys

fig, ax = plt.subplots()
weight = 60

#moveset function
def moveset(Xi,Yi,Thetai,RPM):
    UL = RPM[0]
    UR = RPM[1]
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
    D=0
    while t<1:
        t = t + dt
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt * weight
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt * weight
        Thetan += (r / L) * (UR - UL) * dt
        D=D + math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) *
    dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D, UL, UR

#final path function
def plot_path(Xi,Yi,Thetai,UL,UR,X,Y):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
    while t<1:
        #c = c + 1
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt * weight
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt * weight
        Thetan += (r / L) * (UR - UL) * dt
        plt.plot([Xs, Xn], [Ys, Yn], color="blue")
        if Xn == X and Yn == Y:
            break
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan

#plotting all choices
def plot_choice(Xi,Yi,Thetai,UL,UR,map):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt * weight
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt * weight
        Thetan += (r / L) * (UR - UL) * dt
        if Xs > 0 and Xs < 600 and Ys > 0 and Ys < 200:
            if map[int(Xs),int(Ys),0] == 0:
                plt.plot([Xs, Xn], [Ys, Yn], color="green")
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan

#calculate cost to come for each node
def CalcCost(child, parent):
    xc = child[0]
    yc = child[1]
    xp = parent[0]
    yp = parent[1]
    lx = np.abs(xp-xc)
    ly =np.abs(yp-yc)
    cost = round(np.sqrt(lx**2 + ly**2),1)
    return cost

#calculate cost to go for each node
def CalcCostGo(node, goal):
    x_dist = goal[0] - node[0]
    y_dist = goal[1] - node[1]
    dist = np.sqrt(x_dist**2 + y_dist**2)
    return dist

#create empty map
map_empty = np.zeros([600, 200, 3], dtype=np.uint8)
map_copy = map_empty.copy()

# #assigning start and end points
print('Enter start node x:')
sx = int(input())
print('Enter start node y:')
sy = int(input())
print('Enter starting angle (0 degreees points East):')
sa = int(input())
print('Enter goal node x:')
gx = int(input())
print('Enter goal node y:')
gy = int(input())
print('Enter rpm 1:')
r1 = int(input())
print('Enter rpm 2:')
r2 = int(input())
print('Enter clearance (mm):')
buffer = int(input())

#display values
print('Start Node:',sx, sy, sa)
print('Goal Node:',gx, gy)
print('RPMs:',r1, r2)
print('Clearance:', buffer)

#assign values and dimensions
Xs = [sx, sy, sa, 110,  0, 0]
goal = [gx, gy]
actions=[[r1,r1], [r2,r2],[r1,0],[0,r1],[r1,r2],[r2,r1],[r2,0],[0,r2]]
buffer = int(buffer/10)

# sa = 0
# Xs = [50, 100, 0, 110,  0, 0]
# goal = [580,190]
# actions=[[5,5], [10,10],[5,0],[0,5],[5,10],[10,5],[10,0],[0,10]]
# buffer = 20

#Creating all obstacles on map
#creating square shape dimensions and assigning pixel values to map
x_square_start = 150
x_square_end = 165
y_square_start = 75
y_square_end = 199
for i in range(x_square_start, x_square_end):
    for j in range(y_square_start, y_square_end):
        map_empty[i,j,:] = [239,76,76]

#plotting buffer zone
for i in range(x_square_start-buffer, x_square_end+buffer):
    for j in range(y_square_start-buffer, y_square_end):
            if map_empty[i,j,0] != 239:
                map_empty[i,j,:] = [200,16,16]

#second square
x_square_start = 250
x_square_end = 265
y_square_start = 0
y_square_end = 125
for i in range(x_square_start, x_square_end):
    for j in range(y_square_start, y_square_end):
        map_empty[i,j,:] = [239,76,76]

#plotting buffer zone
for i in range(x_square_start-buffer, x_square_end+buffer):
    for j in range(y_square_start, y_square_end+buffer):
            if map_empty[i,j,0] != 239:
                map_empty[i,j,:] = [200,16,16]

#creating half circle dimensions and assigning pixel values to map
x_hc = 400
hc_r = 50
y_hc = 110
for i in range(x_hc - hc_r, x_hc + hc_r):
    for j in range(y_hc - hc_r, y_hc + hc_r):
        if (i - x_hc) **2 + (j - y_hc)**2 <= hc_r**2:
            map_empty[i,j,:] = [239,76,76]

#plotting buffer zone
for i in range(x_hc - hc_r - buffer, x_hc + hc_r + buffer):
    for j in range(y_hc - hc_r - buffer, y_hc + hc_r + buffer):
        if (i - x_hc) **2 + (j - y_hc)**2 <= (hc_r+buffer)**2 and map_empty[i,j,0] != 239:
            map_empty[i,j,:] = [200,16,16]

#plotting edges
for i in range(0,600):
    for j in range(0,200):
        if i < 6:
            map_empty[i,j,:] = [239,76,76]
        if i > 594:
            map_empty[i,j,:] = [239,76,76]
        if j < 6:
            map_empty[i,j,:] = [239,76,76]
        if j > 193:
            map_empty[i,j,:] = [239,76,76]

dim = (600,200)

v_thresh = 1

#creating visit matrix
vdimx = int(600/v_thresh)
vdimy = int(200/v_thresh)
vdim = (vdimx,vdimy)
V = np.zeros(vdim)

#checking for goalpoint or start point in obstacle
if map_empty[gx,gy,0] != 0:
    print("goal lies in obstacle")
    sys.exit()
if map_empty[sx,sy,0] != 0:
    print("start lies in obstacle")
    sys.exit()

#creating cost to come and total cost matricies
cost_map = np.zeros(vdim)
for i in range(0, 600):
    for j in range(0,200):
        if map_empty[i,j,0] == 0:
            cost_map[int(i/v_thresh),int(j/v_thresh)] = 1E9
        else:
            cost_map[int(i/v_thresh),int(j/v_thresh)] = -1

total_cost_map = np.zeros(vdim)
for i in range(0, 600):
    for j in range(0,200):
        if map_empty[i,j,0] == 0:
            total_cost_map[int(i/v_thresh),int(j/v_thresh)] = 1E9
        else:
            total_cost_map[int(i/v_thresh),int(j/v_thresh)] = -1

#initialize variables
goal_state = 1
OpenList = PriorityQueue()
open_list = []
ClosedList = []
closed_list = []
cost_map[Xs[0],Xs[1]] = 0
goal_thresh = 10
d = 2
parents = {}
rpm = {}
thetas = {}
c = 0
testing_node = []

#start timer
start_time = time.time()

#begin A* algorithm
print("Beginning search")
OpenList.put((0,Xs))
while OpenList and goal_state != 0:

    #start exploring node
    Node_State_i = OpenList.get()[1]

    #make sure node is marked as visited
    V[[int(round(Node_State_i[0]/v_thresh))],[int(round(Node_State_i[1]/v_thresh))]] = 1

    #if node is within goal threshold, end loop
    if Node_State_i[0] > goal[0]-goal_thresh and Node_State_i[1] > goal[1]-goal_thresh and Node_State_i[0] < goal[0]+goal_thresh and Node_State_i[1] < goal[1]+goal_thresh:
        print('SUCCESS')
        goal[0] = Node_State_i[0]
        goal[1] = Node_State_i[1]
        cost2come = cost_map[int(round(Node_State_i[0])),int(round(Node_State_i[1]))]
        cost_map[int(round(Node_State_i[0])),int(round(Node_State_i[1]))] = cost2come
        break

    #get every possible move from moveset
    for action in actions:
        k = moveset(Node_State_i[0],Node_State_i[1],Node_State_i[2],action)
        i = int(round(k[0]))
        j = int(round(k[1]))
        if j < 200 and j > 0 and i < 600 and i > 0:
            if map_empty[i,j,0] == 0:
                testing_node.append(k)

    #loop through each possible move
    for item in testing_node:
        if V[[int(round(item[0]/v_thresh))],[int(round(item[1]/v_thresh))]] == 0:
            if item is not OpenList:
                V[[int(round(item[0]/v_thresh))],[int(round(item[1]/v_thresh))]] == 1
                #c = c+1
                #map_empty[int(round(item[0])),int(round(item[1])),1] = 255 ######
                # if c % 10 == 0:
                #     cv.imwrite("images/Frame%d.jpg"%d, map_empty)
                #     d = d+1
                cost2come = item[3] + cost_map[int(round(Node_State_i[0]/v_thresh)),int(round(Node_State_i[1]/v_thresh))]
                cost_map[int(round(item[0]/v_thresh)),int(round(item[1]/v_thresh))] = cost2come
                cost2go = CalcCostGo(item,goal)
                cost = cost2come + cost2go
                total_cost_map[int(round(item[0]/v_thresh)),int(round(item[1]/v_thresh))] = cost

                parents.setdefault((Node_State_i[0],Node_State_i[1]), [])
                parents[Node_State_i[0],Node_State_i[1]].append([item[0],item[1]])
                
                rpm[item[0],item[1]] = (item[4],item[5])
                thetas[item[0],item[1]] = item[2]
                OpenList.put((cost,item))
        #if node has been visited, check to see if new total cost is less than the current total cost
        else:
            cost2come = item[3] + cost_map[int(round(Node_State_i[0]/v_thresh)),int(round(Node_State_i[1]/v_thresh))]
            cost2go = CalcCostGo(item,goal)
            cost = cost2come + cost2go
            #if new total cost is greater, update with lower value
            if total_cost_map[int(round(item[0]/v_thresh)),int(round(item[1]/v_thresh))] > cost:
                total_cost_map[int(round(item[0]/v_thresh)),int(round(item[1]/v_thresh))] = cost

                parents.setdefault((Node_State_i[0],Node_State_i[1]), [])
                parents[Node_State_i[0],Node_State_i[1]].append([item[0],item[1]])

                rpm[item[0],item[1]] = (item[4],item[5])
                thetas[item[0],item[1]] = item[2]
    testing_node = []
    c = 0

#end timer and print
print("A* search took %s seconds" % (time.time() - start_time))

#Backtracking algorithm
key_list=list(parents.keys())
rk_list = list(rpm.keys())
rv_list = list(rpm.values())
val_list=list(parents.values())

print("Generating Path...")

path = []
spin = []
theta = []
path.append(goal)
node = goal
start = [Xs[0], Xs[1]]
while node != start:
    for list in val_list:
        if node in list:
            ind = val_list.index(list)
            path.append(key_list[ind])
            node = [key_list[ind][0], key_list[ind][1]]
path.reverse()

#get corresponding rpm values
rpm[(Xs[0],Xs[1])] = (10,10)
node = []
while node != goal:
    for node in path:
        if node == goal:
            break
        spin.append(rpm[node])
spin.append((5,5))
spin.append((0,0))

#get corresponding theta values
thetas[(Xs[0],Xs[1])] = sa
node = []
while node != goal:
    for node in path:
        if node == goal:
            break
        theta.append(thetas[node])
theta.append(0.0)
theta.append(0.0)

map_final = cv.rotate(map_empty, cv.ROTATE_90_COUNTERCLOCKWISE)
map_final = cv.flip(map_final, 0)
ax.imshow(map_final, origin = "lower")

#plot paths and algorithm
p = 0
for node in path:
    for action in actions:
        plot_choice(node[0],node[1],theta[p],action[0],action[1], map_empty)
    #plt.savefig('images/frame%d.png'%p, bbox_inches='tight')    
    p = p + 1

for i in range(0,len(path)-1):
    plot_path(path[i][0],path[i][1],theta[i],spin[i+1][0],spin[i+1][1],path[i+1][0],path[i+1][1])
    #plt.savefig('images/frame%d.png'%p, bbox_inches='tight')
    #p = p + 1

plt.title('A* Search With Best Path',fontsize=10)
plt.show()
plt.close()
