import numpy as np
from dataclasses import dataclass
import queue
from enum import Enum
import json
import rospy
from std_msgs.msg import String

pub1 = rospy.Publisher('directions1', String, queue_size=100) 
pub2 = rospy.Publisher('directions2', String, queue_size=100)
rospy.init_node('path_finder_node') 

class Path(Enum):
    gore = 0
    dole = 1
    levo = 2
    desno = 3
    stop = 4

directionsSequence1 = []
directionsSequence1.append("start")
gridQueue1 = queue.Queue()

directionsSequence2 = []
directionsSequence2.append("start")
gridQueue2 = queue.Queue()

@dataclass
class Node:
    x: float
    y: float
    val: float
    visited: bool


def print_matrix(matrix):
    print(np.asarray(matrix))


def findStart(start_flag, matrix):


    start_x = 0
    end_x = 0
    start_y = 0
    end_y = 0

    a = matrix

    for i in range(0,len(matrix)): #find end
        for j in range(0,len(matrix[0])):
            if(a[i][j] == start_flag): #3 je za koordinate cilja
                start_x = j
                break
            else:
                continue
            break
        else:
            continue
        break


    for i in range(0,len(matrix)): #find end
        for j in range(0,len(matrix[0])):
            if(a[i][j] == start_flag and a[i][j+1] == 0): #3 je za koordinate cilja
                end_x = j
                break
            else:
                continue
            break
        else:
            continue
        break

    for i in range(0,len(matrix)): #find end
        for j in range(0,len(matrix[0])):
            if(a[i][j] == start_flag): #3 je za koordinate cilja
                start_y = i

            else:
                continue
            break
        else:
            continue
        break



    for i in range(0,len(matrix)): #find end
        for j in range(0,len(matrix[0])):
            if(a[i][j] == start_flag and a[i][j+1] == 0):
                if(a[i][j] == start_flag and a[i+1][j] == 0): #3 je za koordinate cilja
                    end_y = i
                    break
                else:
                    continue
                break
        else:
            continue
        break

    #print("x1: " + str(start_x) + " x2: " + str(end_x))
    #print("y1: " + str(start_y) + " y2: " + str(end_y))
    mid_x = start_x+end_x
    mid_x/=2

    mid_y = start_y+end_y
    mid_y/=2
    #print("x: " + str(mid_x) + " y: " + str(mid_y))
    return int(mid_x), int(mid_y)

def findEnd(end_flag, matrix):
    return findStart(end_flag, matrix) #ovo je jako debilno ali mrzi me ispravljati

def findObject(objectFlag, matrix):
    return findStart(objectFlag, matrix)

def writeToFile1(p):
    ##print(type(p))
    if(p == "levo"): #matrica je rotirana za 90 stepeni u odnosu na webots teren pa je sve pobrkano
        p = "gore"
    elif(p == "gore"):
        p = "desno"
    elif(p == "dole"):
        p = "levo"
    elif(p ==  "desno"):
        p = "dole"

    global directionsSequence1
    directionsSequence1.append(p)

    with open("/home/milos/meh_ws/src/camera_filter/nodes/directions1.txt", "a") as file_object:
    # Append 'hello' at the end of file
        file_object.write(str(p) + "\n")

def writeToFile2(p):
    ##print(type(p))
    if(p == "levo"): #matrica je rotirana za 90 stepeni u odnosu na webots teren pa je sve pobrkano
        p = "gore"
    elif(p == "gore"):
        p = "desno"
    elif(p == "dole"):
        p = "levo"
    elif(p ==  "desno"):
        p = "dole"

    global directionsSequence2
    directionsSequence2.append(p)

    with open("/home/milos/meh_ws/src/camera_filter/nodes/directions2.txt", "a") as file_object:
    # Append 'hello' at the end of file
        file_object.write(str(p) + "\n")


def checkNeighbours1(grid, x,y, trajectory):

    global directionsSequence1

    p = Path.stop #default state
    if(grid[y][x+1] == grid[y][x] - 1 and grid[y][x+1] != 0):     #grid je 0 ako se na tom mestu nalazi prepreka
        if(grid[y][x] != 1):
            p = Path.desno
            #print(p)
            writeToFile1(p.name)
            checkNeighbours1(grid, x+1, y, trajectory)
    elif(grid[y][x-1] == grid[y][x] - 1 and grid[y][x-1] != 0):
        if(grid[y][x] != 1):
            p = Path.levo
            #print(p)
            writeToFile1(p.name)
            checkNeighbours1(grid, x-1, y, trajectory)
    elif(grid[y-1][x] == grid[y][x] - 1 and grid[y-1][x] != 0):
        if(grid[y][x] != 1):
            p = Path.gore
            #print(p)
            writeToFile1(p.name)
            checkNeighbours1(grid, x, y-1,trajectory)   
    elif(grid[y+1][x] == grid[y][x] - 1 and grid[y+1][x] != 0):
        if(grid[y][x] != 1):
            p = Path.dole
            #print(p)
            writeToFile1(p.name)
            checkNeighbours1(grid, x, y+1,trajectory)

    trajectory[y][x] = 8
    
    return trajectory

def checkNeighbours2(grid, x,y, trajectory):

    global directionsSequence2

    p = Path.stop #default state
    if(grid[y][x+1] == grid[y][x] - 1 and grid[y][x+1] != 0): #grid je 0 ako se na tom mestu nalazi prepreka
        if(grid[y][x] != 1):
            p = Path.desno
            #print(p)
            writeToFile2(p.name)
            checkNeighbours2(grid, x+1, y, trajectory)
    elif(grid[y][x-1] == grid[y][x] - 1 and grid[y][x-1] != 0):
        if(grid[y][x] != 1):
            p = Path.levo
            #print(p)
            writeToFile2(p.name)
            checkNeighbours2(grid, x-1, y, trajectory)
    elif(grid[y-1][x] == grid[y][x] - 1 and grid[y-1][x] != 0):
        if(grid[y][x] != 1):
            p = Path.gore
            #print(p)
            writeToFile2(p.name)
            checkNeighbours2(grid, x, y-1,trajectory)   
    elif(grid[y+1][x] == grid[y][x] - 1 and grid[y+1][x] != 0):
        if(grid[y][x] != 1):
            p = Path.dole
            #print(p)
            writeToFile2(p.name)
            checkNeighbours2(grid, x, y+1,trajectory)

    trajectory[y][x] = 8
    
    return trajectory

with open("/home/milos/meh_ws/src/camera_filter/nodes/directions1.txt", "w") as file_object: #create empty file/owerwrite existing
    file_object.write("start\n") 



with open("/home/milos/meh_ws/src/camera_filter/nodes/directions2.txt", "w") as file_object: #create empty file/owerwrite existing
    file_object.write("start\n")

matrix = np.loadtxt("/home/milos/meh_ws/src/camera_filter/nodes/matrix.txt", dtype=int).tolist()

start_x = 0
start_y = 0

end_x = 0
end_y = 0

object_x = 0
object_y = 0

max_x = len(matrix[0])
max_y = len(matrix)



grid1 = matrix
grid2 = matrix

visited1 = np.zeros((len(matrix), len(matrix[0])), dtype=int).tolist()
grid1 = np.zeros((len(matrix), len(matrix[0])), dtype=int).tolist()
visited2 = np.zeros((len(matrix), len(matrix[0])), dtype=int).tolist()
grid2 = np.zeros((len(matrix), len(matrix[0])), dtype=int).tolist()

start_x, start_y = findStart(2, matrix)
end_x, end_y = findEnd(3, matrix)
object_x, object_y = findObject(4, matrix)

for i in range(0,len(matrix)): #clean matrix
    for j in range(0,len(matrix[0])):
        if(matrix[i][j] != 1):
            matrix[i][j] = 0

for i in range(0,len(matrix)):
    for j in range(0,len(matrix[0])):
        if(matrix[i][j] == 0):
            visited1[i][j] = 0 # 0 for not visited
        else:
            visited1[i][j] = 1 # 1 for visited

for i in range(0,len(matrix)): 
    for j in range(0,len(matrix[0])):
            grid1[i][j] = matrix[i][j]^1

for i in range(0,len(matrix)):
    for j in range(0,len(matrix[0])):
        if(matrix[i][j] == 0):
            visited2[i][j] = 0 # 0 for not visited
        else:
            visited2[i][j] = 1 # 1 for visited            

for i in range(0,len(matrix)): 
    for j in range(0,len(matrix[0])):
            grid2[i][j] = matrix[i][j]^1

##print_matrix(visited)
visited1[object_y][object_x] = 1 #podesi krajnju poziciju na visited
visited2[end_y][end_x] = 1

active_x = object_x
active_y = object_y

active_x1 = end_x
active_y1 = end_y

#krecemo od end_x, end_y u smeru starta kako bi najmanji potencijal bio na startu a najmanji na cilju

#print("start: x:" + str(start_x) + " y: " + str(start_y))
#print("end: x:" + str(end_x) + " y: " + str(end_y))
iterations = 0
iterations1 = 0
gridQueue1.put((object_x, object_y))
gridQueue2.put((end_x, end_y))

while(active_x != start_x or active_y != start_y):
    iterations+=1
    active_x,active_y = gridQueue1.get()
    ##print("active: x:" + str(active_x) + " y: " + str(active_y))

    if(active_x-1 > 0 and visited1[active_y][active_x-1] == 0):
        grid1[active_y][active_x-1]+=grid1[active_y][active_x]
        visited1[active_y][active_x-1] = 1
        gridQueue1.put((active_x-1,active_y))

    if(active_x+1 < max_x and visited1[active_y][active_x+1] == 0):
        grid1[active_y][active_x+1]+=grid1[active_y][active_x]
        visited1[active_y][active_x+1] = 1
        gridQueue1.put((active_x+1,active_y))

    if(active_y-1 > 0 and visited1[active_y-1][active_x] == 0):
        grid1[active_y-1][active_x]+=grid1[active_y][active_x]
        visited1[active_y-1][active_x] = 1
        gridQueue1.put((active_x,active_y-1))

    if(active_y+1 < max_y and visited1[active_y+1][active_x] == 0):
        grid1[active_y+1][active_x]+=grid1[active_y][active_x]
        visited1[active_y+1][active_x] = 1
        gridQueue1.put((active_x,active_y+1))

while(active_x1 != object_x or active_y1 != object_y):
    iterations1+=1
    active_x1,active_y1 = gridQueue2.get()
    ##print("active: x:" + str(active_x) + " y: " + str(active_y))

    if(active_x1-1 > 0 and visited2[active_y1][active_x1-1] == 0):
        grid2[active_y1][active_x1-1]+=grid2[active_y1][active_x1]
        visited2[active_y1][active_x1-1] = 1
        gridQueue2.put((active_x1-1,active_y1))

    if(active_x1+1 < max_x and visited2[active_y1][active_x1+1] == 0):
        grid2[active_y1][active_x1+1]+=grid2[active_y1][active_x1]
        visited2[active_y1][active_x1+1] = 1
        gridQueue2.put((active_x1+1,active_y1))

    if(active_y1-1 > 0 and visited2[active_y1-1][active_x1] == 0):
        grid2[active_y1-1][active_x1]+=grid2[active_y1][active_x1]
        visited2[active_y1-1][active_x1] = 1
        gridQueue2.put((active_x1,active_y1-1))

    if(active_y1+1 < max_y and visited2[active_y1+1][active_x1] == 0):
        grid2[active_y1+1][active_x1]+=grid2[active_y1][active_x1]
        visited2[active_y1+1][active_x1] = 1
        gridQueue2.put((active_x1,active_y1+1))


np.savetxt('/home/milos/meh_ws/src/camera_filter/nodes/potentials_matrix1.txt', grid1, fmt = '%.2d')

        #mozda bi bilo pametno dodati da celijama koje su odmah do zida dodati potencijal jos + 1, da robot ne bi prilazio previse blizu zida, u nekim slucajevima moze biti problem    
np.savetxt('/home/milos/meh_ws/src/camera_filter/nodes/potentials_matrix2.txt', grid2, fmt = '%.2d')

matrixSolved = checkNeighbours1(grid1, start_x, start_y, matrix)
matrixSolved1 = checkNeighbours2(grid2, object_x, object_y, matrix)

#print_matrix(maze)

np.savetxt('/home/milos/meh_ws/src/camera_filter/nodes/matrix_solved1.txt', matrixSolved, fmt = '%.2d')
np.savetxt('/home/milos/meh_ws/src/camera_filter/nodes/matrix_solved2.txt', matrixSolved1, fmt = '%.2d')

with open("/home/milos/meh_ws/src/camera_filter/nodes/directions1.txt", "a") as file_object:
    file_object.write("end\n")

directionsSequence1.append("end")

with open("/home/milos/meh_ws/src/camera_filter/nodes/directions2.txt", "a") as file_object:
    file_object.write("end\n")

directionsSequence2.append("end")

pub1.publish("stop camera node")
for i in range(0 ,len(directionsSequence1)):
#print(directionsSequence1)
    pub1.publish(str(directionsSequence1[i]))

pub2.publish("stop camera node")
for i in range(0 ,len(directionsSequence2)):
#print(directionsSequence2)
    pub2.publish(str(directionsSequence2[i]))
