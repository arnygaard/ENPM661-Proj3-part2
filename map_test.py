import matplotlib.pyplot as plt
import numpy as np
import math
import time
import sys
from queue import PriorityQueue
import cv2 as cv
from matplotlib import transforms
import json

fig, ax = plt.subplots()
weight = 200

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
    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1:
        t = t + dt
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt * weight
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt * weight
        Thetan += (r / L) * (UR - UL) * dt
        #plt.plot([Xs, Xn], [Ys, Yn], color="green")
        #plt.plot([Xs, Xn], [Ys, Yn], color="blue")
        D=D + math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) *
    dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D, UL, UR

def plot_curve(Xi,Yi,Thetai,UL,UR,X,Y):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1 and Xn != X or Yn != Y:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt * weight
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt * weight
        Xd = 0.5*r * (UL + UR) * math.cos(Thetan) * weight
        Yd = 0.5*r * (UL + UR) * math.sin(Thetan) * weight

        Thetan += (r / L) * (UR - UL) * dt
        v = np.sqrt((Xn-Xs)**2+(Yn-Ys)**2)
        s = np.sqrt((Xd)**2+(Yd)**2)
        rz = (r/L)*(UR - UL)
        print(Xn, Yn)
        print("s:",s/345)
        #print("v:",v/20)
        print("rz:",rz)
        # time.sleep(0.1)
        plt.plot([Xs, Xn], [Ys, Yn], color="green")
        # if Xn == X and Yn == Y:
        #     break
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D

def velo(Xi,Yi,Thetai,UL,UR,X,Y):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180
# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1 and Xn != X or Yn != Y:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt * 200
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt * 200

        Thetan += (r / L) * (UR - UL) * dt
        v = np.sqrt((Xn-Xs)**2+(Yn-Ys)**2)
        rz = (r/L)*(UR - UL)
        print(Xn, Yn)
        print(v/20)
        print(rz/2)
        # time.sleep(0.1)
        # if Xn == X and Yn == Y:
        #     break
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D


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

#buffer input
# print('Enter buffer size:')
# buffer = int(input())
buffer = 20

#Creating all obstacles on map
#creating square shape dimensions and assigning pixel values to map
x_square_start = 120 #150
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
x_square_start = 225 #250
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
        if (i - x_hc) **2 + (j - y_hc)**2 <= hc_r**2:  # and j > y_hc:
            map_empty[i,j,:] = [239,76,76]

#plotting buffer zone
for i in range(x_hc - hc_r - buffer, x_hc + hc_r + buffer):
    for j in range(y_hc - hc_r - buffer, y_hc + hc_r + buffer):
        if (i - x_hc) **2 + (j - y_hc)**2 <= (hc_r+buffer)**2 and map_empty[i,j,0] != 239: #and j > y_hc - 5 
            map_empty[i,j,:] = [200,16,16]

for i in range(0,600):
    for j in range(0,200):
        if i < 25:
            map_empty[i,j,:] = [239,76,76]
        if i > 594 - 10:
            map_empty[i,j,:] = [239,76,76]
        if j < 6 + 10:
            map_empty[i,j,:] = [239,76,76]
        if j > 193 - 10:
            map_empty[i,j,:] = [239,76,76]


# #assigning start and end points
# print('Enter step size:')
# l = int(input())
# print('Enter start node x:')
# sx = int(input())
# print('Enter start node y:')
# sy = int(input())
# print('Enter starting angle (0 degreees points East):')
# sa = int(input())
# print('Enter goal node x:')
# gx = int(input())
# print('Enter goal node y:')
# gy = int(input())
# print('Enter goal angle (0 degreees points East):')
# ga = int(input())
# print('Start Node:',sx, sy, sa)
# print('Goal Node:',gx, gy, ga)
# print('Step Size:',l)
# #assign values and dimensions
Xs = [50, 100, 0, 110,  0, 0]
goal = [580,180]
#goal = [200, 120]
#goal_angle = ga
dim = (600,200)
#l = 5
#actions=[[50,50], [55,55],[50,45],[45,50],[50,55],[55,50],[55,45],[45,55]]
actions=[[5,5], [10,10],[5,0],[0,5],[5,10],[10,5]]
#actions=[[100,100], [110,110],[100,80],[80,100],[100,110],[110,100],[110,80],[80,110]]
#actions=[[200,200], [110,110],[200,80],[80,200],[110,200],[200,110],[110,80],[80,110]]
#actions = [[50,50], [50,0], [0,50], [50,100], [100,50], [100,100], [0,100], [100,0]]
#actions=[[50,50], [100,100],[50,0],[0,50],[50,100],[100,50],[100,0],[0,100]]
v_thresh = 1
#creating visit matrix
vdimx = int(600/v_thresh)
vdimy = int(200/v_thresh)
vdim = (vdimx,vdimy)
V = np.zeros(vdim)

##checking for goalpoint or start point in obstacle
# if map_empty[gx,gy,0] != 0:
#     print("goal lies in obstacle")
#     sys.exit()
# if map_empty[sx,sy,0] != 0:
#     print("start lies in obstacle")
#     sys.exit()

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
dupe = 5
#begin A* algorithm
print("Beginning search")
OpenList.put((0,Xs))
while OpenList and goal_state != 0:

    #start exploring node
    Node_State_i = OpenList.get()[1]
    # print(np.sqrt(Node_State_i[0]**2+Node_State_i[1]**2))
    # print(Node_State_i[3])
    # time.sleep(1)
    # print(Node_State_i[4],Node_State_i[5])
    # time.sleep(10)
    #info = cost(Node_State_i[0], Node_State_i[1], Node_State_i[2], )
    #make sure node is marked as visited
    V[[int(round(Node_State_i[0]/v_thresh))],[int(round(Node_State_i[1]/v_thresh))]] = 1

    # for i in range(-dupe,dupe):
    #     for j in range(-dupe,dupe):
    #         if V[[int(round(Node_State_i[0]/v_thresh))+i],[int(round(Node_State_i[1]/v_thresh))+j]] == 0:
    #                 V[[int(round(Node_State_i[0]/v_thresh))],[int(round(Node_State_i[1]/v_thresh))]] = 1


    #if node is within goal threshold, end loop
    if Node_State_i[0] > goal[0]-goal_thresh and Node_State_i[1] > goal[1]-goal_thresh and Node_State_i[0] < goal[0]+goal_thresh and Node_State_i[1] < goal[1]+goal_thresh:
        print('SUCCESS')
        goal[0] = Node_State_i[0]
        goal[1] = Node_State_i[1]
        cost2come = cost_map[int(round(Node_State_i[0])),int(round(Node_State_i[1]))]
        cost_map[int(round(Node_State_i[0])),int(round(Node_State_i[1]))] = cost2come
        break
    #print(Node_State_i)
    #get every possible move from moveset
    #testing_node = cost(Node_State_i[0], Node_State_i[1], Node_State_i[2], )
    for action in actions:
        k = moveset(Node_State_i[0],Node_State_i[1],Node_State_i[2],action)
        #print(k[0],k[1])
        i = int(round(k[0]))
        j = int(round(k[1]))
        if j < 200 and j > 0 and i < 600 and i > 0:
            if map_empty[i,j,0] == 0:
                testing_node.append(k) # (0,0,45) hypothetical start configuration, this dosn't matter for calucating the edges'costs

    #loop through each possible move
    for item in testing_node:
        #if node hasn't been visited, calculate the costs and mark as visited
        # print(V[[int(round(item[0]/v_thresh))],[int(round(item[1]/v_thresh))]])
        # print(int(round(item[0]/v_thresh)),int(round(item[1]/v_thresh)))
        # time.sleep(1)
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

                # rpm.setdefault((Node_State_i[4],Node_State_i[5]), [])
                # rpm[Node_State_i[4],Node_State_i[5]].append([item[4],item[5]])
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
                # rpm.setdefault((Node_State_i[4],Node_State_i[5]), [])
                # rpm[Node_State_i[4],Node_State_i[5]].append([item[4],item[5]])
    testing_node = []
    c = 0

#end timer and print
print("A* search took %s seconds" % (time.time() - start_time))

# #Backtracking algorithm
key_list=list(parents.keys())
rk_list = list(rpm.keys())
rv_list = list(rpm.values())
val_list=list(parents.values())
#print(parents)
#print(rpm)
#print(parents)
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

rpm[(Xs[0],Xs[1])] = (10,10)
node = []
while node != goal:
    for node in path:
        if node == goal:
            break
        spin.append(rpm[node])
spin.append((0,0))
spin.append((0,0))
# spin.append((0,0))
# spin.remove((10,10))

thetas[(Xs[0],Xs[1])] = 0.0
node = []
while node != goal:
    for node in path:
        if node == goal:
            break
        theta.append(thetas[node])
theta.append(0.0)

theta.append(0.0)
# theta.remove(0.0)

# print(len(spin))
# print(len(path))
# print(len(theta))
# print(theta)
#print(spin)

for i in range(0, len(path)-1):
    xi = path[i][0]
    yi = path[i][1]
    xn = path[i+1][0]
    yn = path[i+1][1]
    v = np.sqrt((xn-xi)**2+(yn-yi)**2)
    #print(v)

for item in spin:
    r = 0.038
    L = 0.354
    rz = (r/L)*(item[1]-item[0])
    #print(rz)

#json_string = json.dumps(spin)
with open("rpm.json", "w") as outfile:
    json.dump(spin, outfile)
with open("coords.json","w") as outfile:
    json.dump(path, outfile)
with open("theta.json","w") as outfile:
    json.dump(theta, outfile)
# for item in spin:
#     r = 0.038
#     L = 0.354
#     xd = (r/2)*(item[0]+item[1])* 
#     xd += 0.5*r * (item[0]+item[1]) * math.cos(Thetan)
i = 0
tn = 45
for i in range(0,len(path)-1):
    plot_curve(path[i][0],path[i][1],theta[i],spin[i+1][0],spin[i+1][1],path[i+1][0],path[i+1][1])

for node in path:
    map_empty[int(round(node[0])),int(round(node[1])),1] = 255

#plt.grid()
# tr = transforms.Affine2D().rotate_deg(90)
# ax.imshow(map_empty, transform=tr + ax.transData)
# print(map_empty.shape)
# map_empty = np.reshape(map_empty,(250,600,3))
# print(map_empty.shape)
map_final = cv.rotate(map_empty, cv.ROTATE_90_COUNTERCLOCKWISE)
map_final = cv.flip(map_final, 0)
#np.transpose(map_empty, (1, 0, 2))
ax.imshow(map_final, origin = "lower")
#ax.set_aspect('equal')

# plt.xlim(0,50)
# plt.ylim(0,50)
plt.title('How to plot a vector in matplotlib ?',fontsize=10)
plt.show()
plt.close()

# #mark all visited nodes as green
# for i in range(0, V.shape[0]):
#     for j in range(0,V.shape[1]):
#         for k in range(0,V.shape[2]):
#             if V[i][j][k] == 1:
#                 map_empty[i,j,1] = 255

# image = map_empty

# for i in range(0,len(path) - 1):
#     x1 = path[i][0]
#     y1 = path[i][1]
#     x2 = path[i+1][0]
#     y2 = path[i+1][1]
#     image = cv.arrowedLine(image, (y1,x1), (y2,x2), color=(0,0,255), thickness=1) 

# #creating and displaying final map
# map_final = cv.rotate(image, cv.ROTATE_90_COUNTERCLOCKWISE)
# map_final = cv.cvtColor(map_final, cv.COLOR_RGB2BGR)

# cv.imshow("A* Search with best path",map_final)
# cv.waitKey(0)
# cv.destroyAllWindows()