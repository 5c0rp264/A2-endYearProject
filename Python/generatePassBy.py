from collections import defaultdict
from heapq import *
import re

##map of all node to find wich way does the robot have to take
nodesAsMap = [[4,8,10,13,16], 
                [3,7,9,0,0],
                [2,0,0,12,15],
                [0,6,0,11,0],
                [1,5,0,0,14]]
##what to write inside of the Arduino library
defaultDataBefore = "#ifndef pathToTake_H\n#define pathToTake_H \nint8_t pathToTake[index]=shouldBePlaceHere; \n#endif"


##function to find location of a node inside of the map, it return [x,y] position
def find(intToFind):
    for i in list(range(len(nodesAsMap))):
        for j in list(range(len(nodesAsMap[i]))):
            if nodesAsMap[i][j] == intToFind :
                return [j,i];
        

def dijkstra(edges, f, t):
    g = defaultdict(list)
    for l,r,c in edges:
        g[l].append((c,r))##add to the g dictionnary each edge and distance to other one

    q, seen, mins = [(0,f,())], set(), {f: 0} ## declare q as heap, seen as a set of already checked node and mins as a set of minimal distance  between node
    while q:
        (cost,v1,path) = heappop(q) ##get value inside heap as the cost to go here, the path  and the node
        if v1 not in seen: ## do not lookup this node if already seen
            seen.add(v1)## add node to seen not to do this job again
            path = (v1, path)##add path if shortest and inside of the way f and t
            if v1 == t: return (cost, path) ##if we are at t it means we found the shortest path so return

            for c, v2 in g.get(v1, ()):
                if v2 in seen: continue
                prev = mins.get(v2, None)
                next = cost + c ##add cost  wich represent the distance
                if prev is None or next < prev:
                    mins[v2] = next
                    heappush(q, (next, v2, path))

    return float("inf")##return infinity if the whole heap is done and no shortest path find

if __name__ == "__main__": ## good habit
    edges = [('1', '2', 4),##declaring each connection, both way inside the map
            ('1', '5', 3),
            ('2', '3', 4),
            ('2', '12', 12),
            ('3', '4', 2),
            ('3', '7', 3),
            ('4', '8', 3),
            ('5', '6', 2),
            ('5', '14', 12),
            ('6', '11', 9),
            ('7', '8', 2),
            ('7', '9', 6),
            ('8', '10', 6),
            ('9', '10', 2),
            ('10', '13', 3),
            ('11', '12', 2),
            ('12', '13', 4),
            ('12', '15', 3),
            ('13', '16', 3),
            ('14', '15', 4),
            ('15', '16', 4),
            ('2', '1', 4),
            ('3', '2', 4),
            ('4', '3', 2),
            ('5', '1', 3),
            ('6', '5', 2),
            ('7', '3', 3),
            ('8', '7', 2),
            ('8', '4', 3),
            ('9', '7', 6),
            ('10', '9', 2),
            ('10', '8', 6),
            ('11', '6', 9),
            ('12', '2', 12),
            ('12', '11', 2),
            ('13', '10', 3),
            ('13', '12', 4),
            ('14', '5', 12),
            ('15', '12', 3),
            ('16', '13', 3),
            ('15', '14', 4),
            ('16', '15', 4)]

    print ("======= Dijkstra =======")
    print (edges)
    source = str(input("Please enter where you want to start:  "))
    passBy = str(input("Please enter where you want to passBy:  "))
    destination = str(input("Please enter where you want to finish:  "))
    dataOutput = str(dijkstra(edges, passBy, source))
    dataOutput2 = str(dijkstra(edges, destination, passBy))
    dataOutput = re.sub(r'\(',r'', dataOutput) ##treating dataOutput as this is a pth and is is outpouted weirdly
    dataOutput = re.sub(r'\)',r'', dataOutput)
    dataOutput = dataOutput.replace("'", "")
    dataOutput2 = re.sub(r'\(',r'', dataOutput2) ##treating dataOutput as this is a pth and is is outpouted weirdly
    dataOutput2 = re.sub(r'\)',r'', dataOutput2)
    dataOutput2 = dataOutput2.replace("'", "")
    nodeToGo = dataOutput.split(",")
    nodeToGo2 = dataOutput2.split(",")
    del nodeToGo[0]
    del nodeToGo[len(nodeToGo)-1]
    del nodeToGo2[0]
    del nodeToGo2[0]
    del nodeToGo2[len(nodeToGo2)-1]
    for nodeToAdd in nodeToGo2 :
        nodeToGo.append(nodeToAdd)
    print(nodeToGo)
    order = "{"
    orderNum = 0
    for node in list(range(len(nodeToGo))):
        if node != 0 and node != len(nodeToGo)-1 :
            ##print("node : " + str(node))
            previous = find(int(nodeToGo[node-1]))
            current = find(int(nodeToGo[node]))
            ##print("nextOneIndex: "+str(int(nodeToGo[node+1])))
            nextOne = find(int(nodeToGo[node+1]))
##            print("previous : " + str(previous))
##            print("current : " + str(current))
##            print("nextOne : " + str(nextOne))
           ## print(previous)
            if ((int(nodeToGo[node])==int(passBy)) and (int(nodeToGo[node-1])==int(nodeToGo[node+1]))):
                print ("reverse everything !!!!")
                order+="2"
            elif (previous[0] == current[0] and current[0]==nextOne[0]): ## all if here are to tell where to go checking every position
                print ("go forward")
                order+="5"
            elif (previous[1]==current[1] and current[1]==nextOne[1]):
                print ("go forward")
                order+="5"
            elif (previous[0]==current[0] and previous[1]>current[1] and current[0]<nextOne[0] and current[1]==nextOne[1]):
                print("go right")
                order+="3"
            elif (previous[0]==current[0] and previous[1]>current[1] and current[0]>nextOne[0] and current[1]==nextOne[1]):
                print("go left")
                order+="1"
            elif (previous[0]==current[0] and previous[1]<current[1] and current[0]>nextOne[0] and current[1]==nextOne[1]):
                print("go right")
                order+="3"
            elif (previous[0]==current[0] and previous[1]<current[1] and current[0]<nextOne[0] and current[1]==nextOne[1]):
                print("go left")
                order+="1"
            elif (previous[0]<current[0] and previous[1]==current[1] and current[0]==nextOne[0] and current[1]<nextOne[1]):
                print("go right")
                order+="3"
            elif (previous[0]<current[0] and previous[1]==current[1] and current[0]==nextOne[0] and current[1]>nextOne[1]):
                print("go left")
                order+="1"
            elif (previous[0]>current[0] and previous[1]==current[1] and current[0]==nextOne[0] and current[1]>nextOne[1]):
                print("go right")
                order+="3"
            elif (previous[0]>current[0] and previous[1]==current[1] and current[0]==nextOne[0] and current[1]<nextOne[1]):
                print("go left")
                order+="1"
            order+=","
            orderNum+=1
    if orderNum != 0 :
        order = order[:-1]
    order+="}"
    print(order)
    f = open(r'C:\Users\quent\Documents\Arduino\libraries\NeverMind\\pathToTake.h', "w")
    defaultDataBefore = defaultDataBefore.replace("shouldBePlaceHere",order)
    defaultDataBefore = defaultDataBefore.replace("index",str(orderNum))
    f.write(defaultDataBefore)
    f.close()  
    print("Should be place from" + str(nodeToGo[0]) + " to " + str(nodeToGo[1]))
    print("fullPath : " + str(nodeToGo))







