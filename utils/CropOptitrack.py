import rosbag as rb
import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import Float64, Float64MultiArray

#read files

sample_no = 0

string = "optitrack_drone_"+str(sample_no)+".csv"

OptiTrackData = pd.read_csv(string)
IMUbag = rb.Bag("IMU.bag")
DVSbag = rb.Bag("DVS.bag")

frequency = 250


threshold = 0.005 #threshold for movement in optitrack
check = False
movement = []

StartFound = False
rollingaverage = []


AccThreshold = 2 #threshold for acceleration in rosbag

acc = []
rostime = []
first = True
end = False
begin  = False


#for the OT

InitialPos = np.array([OptiTrackData["X"][1], OptiTrackData["Y"][1], OptiTrackData["Z"][1]])


for i in range(1, len(OptiTrackData.index)):
    
    CurrentPos = np.array([OptiTrackData["X"][i], OptiTrackData["Y"][i], OptiTrackData["Z"][i]])
    
    Movement = abs(CurrentPos - InitialPos)
    
    movement.append(CurrentPos)

    if ((sum(Movement) > threshold) and not StartFound): #if moving, start
        StartIndex = i  # find index in csv
        StartFound = True
        
        
    
InitialPos = np.array([OptiTrackData["X"][len(OptiTrackData.index)-1], OptiTrackData["Y"][len(OptiTrackData.index)-1], OptiTrackData["Z"][len(OptiTrackData.index)-1]])
    

for i in range(len(OptiTrackData.index)-1,1,-1):
    CurrentPos = np.array([OptiTrackData["X"][i], OptiTrackData["Y"][i], OptiTrackData["Z"][i]])
    Movement = abs(CurrentPos - InitialPos)
    if ((sum(Movement) > threshold)): #if moving, start
        EndIndex = i  # find index in csv        
        break
        
ElapsedTime = OptiTrackData["Time"][EndIndex] - OptiTrackData["Time"][StartIndex] # for ns to s
OptiTrackDataShort = OptiTrackData[StartIndex:EndIndex]
OptiTrackDataShort.to_csv("optitrack_short.csv")


## TO DO: WRITE ROSBAG

MovingAverage = []
AverageAcc = []
NewIMUBag = []

for msg in IMUbag.read_messages():
    rostime.append(msg.timestamp.to_sec())
    acc.append([msg.message.linear_acceleration.x, msg.message.linear_acceleration.y, msg.message.linear_acceleration.z] )
    
    MovingAverage.append(msg.message.linear_acceleration.y)
    
    if len(MovingAverage) > 10:
        MovingAverage.pop(0)
    
    Av = sum(MovingAverage)/len(MovingAverage)
    
    AverageAcc.append(Av)
    
    if first: 
        StandardValue = msg.message.linear_acceleration.y #set standard value ~= g
        print(StandardValue)
        first = False
        
#if beginning of movement has not been found yet AND it starts now
    if abs(Av-StandardValue) > AccThreshold and not begin: 
        RosBeginTime = msg.timestamp.to_sec() 
        begin = True
        
    if begin: 
        NewIMUBag.append(msg)
    
    if begin and msg.timestamp.to_sec()-RosBeginTime >= ElapsedTime:
        RosEndTime = msg.timestamp.to_sec()
        break



#calc time    
print("Elapsed time in Optitrack: "+str(ElapsedTime))
print("Total Optitrack time: "+str(OptiTrackData["Time"][len(OptiTrackData.index)-1]-OptiTrackData["Time"][0]))
print("Short time in bag: "+str(RosEndTime - RosBeginTime))
print("Total time in bag: "+str(IMUbag.get_end_time()-IMUbag.get_start_time()))

with rb.Bag('NewBag.bag','w') as NewBag:
    for msg in NewIMUBag:
        
        NewBag.write('/dvs/imu', msg.message, msg.timestamp)
    print("wrote IMUbag")

    for msg in DVSbag:
         if msg.timestamp.to_sec() >= RosBeginTime and msg.timestamp.to_sec() <= RosEndTime:
             NewBag.write('/dvs/events', msg.message, msg.timestamp)
    print("wrote DVSbag")
    
    for row in range(OptiTrackDataShort.shape[0]):
        timestamp = rospy.Time.from_sec(OptiTrackData['Time'][row] + RosBeginTime)
        # TODO: write custom msg


        msg = Float64MultiArray()
        msg.data = (OptiTrackData['X'][row],OptiTrackData['Y'][row],OptiTrackData['Z'][row],OptiTrackData['w'][row],OptiTrackData['x'][row],OptiTrackData['y'][row],OptiTrackData['z'][row])
        NewBag.write('/optitrack', msg, timestamp)
    
    print("wrote OT")
        
        



plt.figure(1)
plt.plot(movement)
plt.axvline(x=StartIndex)
plt.axvline(x=EndIndex)


plt.figure(2)
plt.plot(rostime,acc)
plt.axvline(x=RosBeginTime)
plt.axvline(x=RosEndTime)

plt.show()



