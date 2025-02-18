import numpy as np
import math

class cell:
    def __init__(self):
        self.parent = [-1,-1]
        self.f = math.inf
        self.g = math.inf
        self.h = math.inf

def isValid(coordinates):
    return (coordinates[0] in range(dims[0]) and coordinates[1] in range(dims[1]) and maze[coordinates[0]][coordinates[1]])

def grassFire(cells,src):
    tempList = []
    tempList.append(src)
    while(len(tempList)):
        temp = tempList[0]
        x,y = temp
        tempList.pop(0)
        if(isValid([x-1,y]) and cells[x-1][y].h == math.inf):
            cells[x-1][y].h = min(cells[x][y].h + 1.0,cells[x-1][y].h)
            tempList.append([x-1,y])
        if(isValid([x,y-1]) and cells[x][y-1].h == math.inf):
            cells[x][y-1].h = min(cells[x][y].h + 1.0,cells[x][y-1].h)
            tempList.append([x,y-1])
        if(isValid([x,y+1]) and cells[x][y+1].h == math.inf):
            cells[x][y+1].h = min(cells[x][y].h + 1.0,cells[x][y+1].h)
            tempList.append([x,y+1])
        if(isValid([x+1,y]) and cells[x+1][y].h == math.inf):
            cells[x+1][y].h = min(cells[x][y].h + 1.0,cells[x+1][y].h)
            tempList.append([x+1,y])

def calculateHeuristics(coordinates,dest):
    return abs(coordinates[0]-dest[0]) + abs(coordinates[1]-dest[1])

def tracePath(cells,dest):
    x,y = dest
    path = []
    while(not (cells[x][y].parent == [x,y])):
        path.append([x,y])
        x,y = cells[x][y].parent
    return path[::-1]


def astarAlgorithm(src,dest):
    if(isValid(src) == False):
        print("Invalid Source")
        return
    if(isValid(dest) == False):
        print("Invalid Destination")
        return
    if(src == dest):
        print("Already at the destination")
        return
    closedList = np.zeros(dims)
    cells = []
    for i in range(dims[0]):
        temp = []
        for j in range(dims[1]):
            temp.append(cell())
        cells.append(temp)
    x,y = src
    cells[x][y].parent=[x,y]
    cells[x][y].f = 0.0
    cells[x][y].g = 0.0
    cells[x][y].h = 0.0
    grassFire(cells,src)
    openList = []
    openList.append([0,src])
    foundDest = False
    while(len(openList)):
        openList.sort(key = lambda li: li[0])
        temp = openList[0]
        x,y = temp[1]
        openList.pop(0)
        closedList[x][y] = 1

        x -= 1
        y -= 1
        if(isValid([x,y])):
            if(dest == [x,y]):
                cells[x][y].parent = temp[1]
                foundDest = True
                break
            if(not  closedList[x][y]):
                gNew = cells[temp[1][0]][temp[1][1]].g+1.0
                hNew = cells[x][y].h
                fNew = gNew + hNew
                if(cells[x][y].f > fNew):
                    openList.append([fNew,[x,y]])
                    cells[x][y].parent = temp[1]
                    cells[x][y].f = fNew
                    cells[x][y].g = gNew
        y += 1
        if (isValid([x, y])):
            if (dest == [x, y]):
                cells[x][y].parent = temp[1]
                foundDest = True
                break
            if (not closedList[x][y]):
                gNew = cells[temp[1][0]][temp[1][1]].g + 1
                hNew = cells[x][y].h
                fNew = gNew + hNew
                if (cells[x][y].f > fNew):
                    openList.append([fNew, [x, y]])
                    cells[x][y].parent = temp[1]
                    cells[x][y].f = fNew
                    cells[x][y].g = gNew
        y += 1
        if (isValid([x, y])):
            if (dest == [x, y]):
                cells[x][y].parent = temp[1]
                foundDest = True
                break
            if (not closedList[x][y]):
                gNew = cells[temp[1][0]][temp[1][1]].g + 1
                hNew = cells[x][y].h
                fNew = gNew + hNew
                if (cells[x][y].f > fNew):
                    openList.append([fNew, [x, y]])
                    cells[x][y].parent = temp[1]
                    cells[x][y].f = fNew
                    cells[x][y].g = gNew
        x += 1
        y -= 2
        if (isValid([x, y])):
            if (dest == [x, y]):
                cells[x][y].parent = temp[1]
                foundDest = True
                break
            if (not closedList[x][y]):
                gNew = cells[temp[1][0]][temp[1][1]].g + 1
                hNew = cells[x][y].h
                fNew = gNew + hNew
                if (cells[x][y].f > fNew):
                    openList.append([fNew, [x, y]])
                    cells[x][y].parent = temp[1]
                    cells[x][y].f = fNew
                    cells[x][y].g = gNew
        y += 2
        if (isValid([x, y])):
            if (dest == [x, y]):
                cells[x][y].parent = temp[1]
                foundDest = True
                break
            if (not closedList[x][y]):
                gNew = cells[temp[1][0]][temp[1][1]].g + 1
                hNew = cells[x][y].h
                fNew = gNew + hNew
                if (cells[x][y].f > fNew):
                    openList.append([fNew, [x, y]])
                    cells[x][y].parent = temp[1]
                    cells[x][y].f = fNew
                    cells[x][y].g = gNew
        x += 1
        y -= 2
        if (isValid([x, y])):
            if (dest == [x, y]):
                cells[x][y].parent = temp[1]
                foundDest = True
                break
            if (not closedList[x][y]):
                gNew = cells[temp[1][0]][temp[1][1]].g + 1
                hNew = cells[x][y].h
                fNew = gNew + hNew
                if (cells[x][y].f > fNew):
                    openList.append([fNew, [x, y]])
                    cells[x][y].parent = temp[1]
                    cells[x][y].f = fNew
                    cells[x][y].g = gNew
        y += 1
        if (isValid([x, y])):
            if (dest == [x, y]):
                cells[x][y].parent = temp[1]
                foundDest = True
                break
            if (not closedList[x][y]):
                gNew = cells[temp[1][0]][temp[1][1]].g + 1
                hNew = cells[x][y].h
                fNew = gNew + hNew
                if (cells[x][y].f > fNew):
                    openList.append([fNew, [x, y]])
                    cells[x][y].parent = temp[1]
                    cells[x][y].f = fNew
                    cells[x][y].g = gNew
                    #cells[x][y].h = hNew
        y += 1
        if (isValid([x, y])):
            if (dest == [x, y]):
                cells[x][y].parent = temp[1]
                foundDest = True
                break
            if (not closedList[x][y]):
                gNew = cells[temp[1][0]][temp[1][1]].g + 1
                hNew = cells[x][y].h
                fNew = gNew + hNew
                if (cells[x][y].f > fNew):
                    openList.append([fNew, [x, y]])
                    cells[x][y].parent = temp[1]
                    cells[x][y].f = fNew
                    cells[x][y].g = gNew
    np.transpose(cells)

    if(foundDest):
        return tracePath(cells,dest)
    else:
        print("Failed to reach the destination")

def convertCoordinates(coordinates):
	coordinates[0] = math.floor(4*(coordinates[0]+11))
	coordinates[1] = math.floor(4*(coordinates[1]+11))
	return

def modify_path(src,path):
    if(len(path) < 3):
        return path
    tempList = []
    i = 1
    prev = src
    while(i < len(path)):
        fromX = min(prev[0],path[i][0])
        toX = prev[0]+path[i][0]-fromX+1
        fromY = min(prev[1],path[i][1])
        toY = prev[1]+path[i][1]-fromY+1
        total = 0
        for x in range(fromX,toX):
            for y in range(fromY,toY):
                total += maze[x][y]
        if(total < (toX-fromX)*(toY-fromY)):
            prev = path[i-1]
            i += 1
        else:
            path.pop(i-1)

    for i in range(len(path)):
        x,y = path[i]
        if(maze[x][y-1] == 0):
            path[i][1] += 2
        elif(maze[x][y+1] == 0):
            path[i][1] -= 2
        if(maze[x-1][y] == 0):   
            path[i][0] += 2
        elif(maze[x+1][y] == 0): 
            path[i][0] -= 2
    return path

def calculateShortestPath(src,dest):

    global maze
    maze = [[1, 1, 1, 1, 1],
        [1 ,1 ,1 ,0 ,1],
        [0, 0, 1, 0, 1],
        [1 ,1 ,1 ,0 ,1],
        [1, 0, 1, 0, 1],
        [1 ,0 ,1 ,1 ,1],
        [1, 0, 1, 0, 1],
        [1 ,1 ,1 ,1 ,1 ]]
    mat = np.ones((89, 89))
    for y in range(89):
        mat[0, y] = 0
        mat[88, y] = 0
    for x in range(89):
        mat[x, 0] = 0
        mat[x, 88] = 0

    for y in range(24, 65):
        mat[12, y] = 0

    for y in range(24, 65):
        mat[76, y] = 0

    for y in range(8, 33):
        mat[28, y] = 0

    for y in range(56, 82):
        mat[28, y] = 0

    for y in range(8, 33):
        mat[60, y] = 0

    for y in range(56, 82):
        mat[60, y] = 0

    for x in range(36, 53):
        mat[x, 20] = 0

    for x in range(36, 53):
        mat[x, 68] = 0
    maze = mat

    global dims
    dims = [89,89]
    convertCoordinates(src)
    convertCoordinates(dest)
    path = astarAlgorithm(src, dest)
    path = modify_path(src,path)

    return path[:]
    x = [x[0] for x in path[:-1]]
    y = [y[1] for y in path[:-1]]

