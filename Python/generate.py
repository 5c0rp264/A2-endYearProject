from collections import defaultdict
from heapq import *
import re


nodesAsMap = [[4,8,10,13,16],
                [3,7,9,0,0],
                [2,0,0,12,15],
                [0,6,0,11,0],
                [1,5,0,0,14]]

defaultDataBefore = "#ifndef pathToTake_H\n#define pathToTake_H \nint8_t pathToTake[index]=shouldBePlaceHere; \n#endif"

def find(intToFind):
    for i in list(range(len(nodesAsMap))):
        for j in list(range(len(nodesAsMap[i]))):
            if nodesAsMap[i][j] == intToFind :
                return [j,i];
        

def dijkstra(edges, f, t):
    g = defaultdict(list)
    for l,r,c in edges:
        g[l].append((c,r))

    q, seen, mins = [(0,f,())], set(), {f: 0}
    while q:
        (cost,v1,path) = heappop(q)
        if v1 not in seen:
            seen.add(v1)
            path = (v1, path)
            if v1 == t: return (cost, path)

            for c, v2 in g.get(v1, ()):
                if v2 in seen: continue
                prev = mins.get(v2, None)
                next = cost + c
                if prev is None or next < prev:
                    mins[v2] = next
                    heappush(q, (next, v2, path))

    return float("inf")

if __name__ == "__main__":
    edges = [('1', '2', 4),
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
            ('15', '14', 4),
            ('16', '15', 4)]

    print ("=== Dijkstra ===")
    print (edges)
    source = str(input("Please enter where you want to start:  "))
    destination = str(input("Please enter where you want to finish:  "))
    dataOutput = str(dijkstra(edges, destination, source))
    dataOutput = re.sub(r'\(',r'', dataOutput)
    dataOutput = re.sub(r'\)',r'', dataOutput)
    dataOutput = dataOutput.replace("'", "")
    nodeToGo = dataOutput.split(",")
    del nodeToGo[0]
    del nodeToGo[len(nodeToGo)-1]
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
            if (previous[0] == current[0] and current[0]==nextOne[0]):
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







