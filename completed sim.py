import math
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
    
def angle(coord1,coord2):
    """This works out the angle between two coordinates"""
    deltax=coord2[0]-coord1[0]
    deltay=coord2[1]-coord1[1]
    ang=math.atan2(deltax,deltay)
    return ang

def distance(coord1,coord2):
    """This works out the distance between two coordinates"""
    deltax=coord2[0]-coord1[0]
    deltay=coord2[1]-coord1[1]
    dist=math.sqrt((deltax)**2+(deltay)**2)
    return dist

def drive(linear, angular, length):
    """This moves the simulated rover forward, or turns it"""
    global y #global values are used to represents position and bearing
    global x
    global bearing
    global t
    for i in range(10):
        y+=math.cos(bearing)*linear*length/10 #this moves the Rover in x and y
        x+=math.sin(bearing)*linear*length/10
        if drift=="y":  #if drift is included in the sim, the bearing is modified here
            bearing +=np.pi/180
    bearing += angular*length
    bearing=bearing%(2*(math.pi))
    t+=length



def navigate(destination):
    """This navigates the simulated Rover to a location"""
    count=0
    while True:
        location1=(x, y)
        route=angle(location1, destination) #the angle between the current location and destination is worked out
        difference=bearing-route #the difference between the two is determined
        if abs(difference)>(1/5)*math.pi: #if the difference is large enough, the rover is rotated. The tolerance has been decreased to produce clearer results
            direction=difference/abs(difference)
            length=abs(difference)
            drive(0.0,-direction,length)
            turnlist.append(location1)
        if distance(location1,destination)<2: #if the rover is closer than 2 metres, the rover drives for a smaller time
            drive(.2,0.0,distance(location1,destination)/.2)
        else:
            drive(.2,0.0,10) #this moves the rover forward
        location2=(x,y)
        xarray.append(x)
        yarray.append(y) #the location is recorded
        if distance(location2, destination)<.25: #if close enough, the rover will stop now
            break
        count+=1
        if count>10000:
            break

bearing=-1
x=0
y=0
t=0
xarray=[0]
yarray=[0]
turnlist=[]
def mapper(corner1,corner2,corner3,corner4,num,pickup):
    """this travels to every point within 4 corners"""
    coords=np.array([corner1,corner2,corner3,corner4]) 
    dist1=[]
    for i in range(4): #this works out which corner is furtherest away, and hence which corner is not connected to the first corner
        length=distance(coords[0],coords[i])
        dist1.append(length)
    maxi=dist1.index(max(dist1))
    zero=dist1.index(0)
    
    del dist1[zero] #this discards the point of the corner we are starting at 
    mini=dist1.index(min(dist1)) #this works out the closest of the two connecting corners
    x1=np.linspace(coords[0,0],coords[mini+1,0],num) #the x and y space between the closest corner is divided
    y1=np.linspace(coords[0,1],coords[mini+1,1],num)
    line1=np.column_stack([x1,y1]) #a line is drawn between the two points (the origin and the closest corner)
    
    indexnums=[0,1,2,3]
    indexnums.remove(zero)
    indexnums.remove(maxi)
    indexnums.remove(mini+1)
    medium=indexnums.pop(0)#this returns the other conneting corner
    
    x2=np.linspace(coords[medium,0],coords[maxi,0],num) 
    y2=np.linspace(coords[medium,1],coords[maxi,1],num)
    line2=np.column_stack([x2,y2])# a line is drawn between the other connecting corner and the non connecting corner
    travelcoords=np.empty((num**2,2)) #an empty array is created with enough spaces to hold all points
    for i in range(num): #two lines have been created, on each side of the shape. The following code divides these lines up, and connects the points between them, 
        #creating lines across the shape, and ordering the points such that the array snakes around the shape
        xline=np.linspace(line1[i,0],line2[i,0],num)
        yline=np.linspace(line1[i,1],line2[i,1],num)
        line=np.column_stack([xline,yline])
        if i%2==0:
            for j in range(num):
                travelcoords[num*i+j]=line[j]
        if i%2!=0:
            for j in range(num):
                travelcoords[num*i+j]=line[num-j-1]
    
   
    if pickup=="y":
        zonex=20
        zoney=50
        width=5
        height=20
        circlepos=(zonex,zoney)
        navigate(coords[0])
        removed=0
        for i in range(num**2+1):
            navigate(travelcoords[i])
            if i== num**2+2-removed:
                position=len(xarray)
            if x<25 and x>20:
                if y>50 and y<70:
                    extract=12-i%12
                    deletelist=[i]
                    for j in range(2*(extract-1)):
                        deletelist.append(i+j)               
                    sliced=np.asarray(travelcoords[i:(i+2*extract+1)])
                    travelcoords=np.delete(travelcoords,deletelist, axis=0)
                    travelcoords=np.concatenate((travelcoords,sliced))
    
                    removed+=2*extract+1
        return position,circlepos, width, height, travelcoords
    else: 
        navigate(coords[0])#this navigates to the points
        for i in range(num**2):
            navigate(travelcoords[i])




drift=input("To include drift in simulations, enter y. Enter n to not include drift:")
menuinput=input("Type n to run a simulation of the navigate script, or m to run a simulation of the mapper script. To simulate the rover avoiding pickup, press p:")
if menuinput=="n":
    destinationx=float(input("Enter a destination x coordinate:"))
    destinationy=float(input("Enter a destination y coordinate:"))
    destination=(destinationx, destinationy)
    navigate(destination)
    turnarray=np.asarray(turnlist)
    plt.scatter(destination[0],destination[1], c="red", s=100, zorder=10)
    plt.plot(xarray,yarray,"-o")
    plt.scatter(turnarray[:,0], turnarray[:,1], c="orange", zorder=9)
    plt.axis('scaled')
    
    plt.show()

if menuinput=="m":
    mapdest=input("To use predetermined corners, enter c. To enter your own, press n:")
    if mapdest=="c":
        corner=[(0,0),(100,10),(10,100),(100,100)]
    if mapdest=="n":
        print("Enter your corner coordinates. The shape should be approximately rectangular (the two connecting corners to each corner should be closer than the opposite one).")
        for i in range(4):
            mapdestx=float(input("Enter the x coordinate for corner",i))
            mapdesty=float(input("Enter the y coordinate for corner",i))
            corner[i]=(mapdestx,mapdesty)
    mapper(corner[0],corner[1],corner[2],corner[3], 12,"n")
    plt.plot(xarray, yarray)
    corners=np.array([corner[0], corner[1], corner[2], corner[3]])
    plt.scatter(corners[:,0],corners[:,1], c="red")
    plt.show()
if menuinput=="p":
    corner=[(0,0),(100,10),(10,100),(100,100)]
    position, circlepos, width, height, travelcoords=mapper(corner[0],corner[1],corner[2],corner[3], 12,"y")
    fig=plt.figure()

    ax1=plt.axes(xlim=(-10,110), ylim=(-10,110))
    ax1.add_patch(patches.Rectangle(circlepos,width, height))
    plt.plot(xarray[:position], yarray[:position], "g")
    #plt.plot(xarray[position:], yarray[position:], "r")
    #plt.scatter(travelcoords[:,0],travelcoords[:,1])
    corners=np.array([corner[0], corner[1], corner[2], corner[3]])
    plt.scatter(corners[:,0],corners[:,1], c="red")