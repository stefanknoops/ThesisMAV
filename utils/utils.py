# This file is part of the Loihi repository - MAVLab TU Delft
#
# MIT License
#
# Copyright (c) 2021 Julien Dupeyroux
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# @author Julien Dupeyroux

# ----------------------------------------------------------------------------
# Import modules
# ----------------------------------------------------------------------------
import rosbag
from PIL import Image
import subprocess
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

# Full resolution
DIMX = 240
DIMY = 180
# Half resolution (to remove spike rate issue between columns)
hDIMX = 120
hDIMY = 90

# ----------------------------------------------------------------------------
# Function for Python version checking (for Loihi only)
# ----------------------------------------------------------------------------
def check_python_version():
    if not ((3, 5, 2) <= sys.version_info[:3] < (3, 6, 0)):
        pyversion = ".".join("%d" % v for v in sys.version_info[:3])
        raise EnvironmentError(
            "NxSDK has .pyc files that only work on Python 3.5.2 through 3.5.5. "
            "You are running version %s." % pyversion)

# ----------------------------------------------------------------------------
# Function for IMU rosbag data loading
# ----------------------------------------------------------------------------
def extract_imu_data(data_path):
    print(" .. Loading the IMU rosbag")
    # Rosbag
    IMU_BAG = rosbag.Bag(data_path)
    # Variables for IMU data
    imu_time = []
    imu_acceleration = []
    imu_angular_velocity = []
    # Data loading
    flag = False
    init_time = -1
    for topic, msg, t in IMU_BAG.read_messages(topics=['data']):
        if not flag:
            flag = True
            init_time = t.to_nsec()*1e-9
        imu_time.append(t.to_nsec()*1e-9 - init_time)
        imu_angular_velocity.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        imu_acceleration.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    IMU_BAG.close()
    return imu_time, imu_acceleration, imu_angular_velocity

# ----------------------------------------------------------------------------
# Function for IMU rosbag data conversion into csv
# ----------------------------------------------------------------------------
def imu_to_csv(data_path, sample_path):
    print(" .. Converting the IMU rosbag into .csv")
    # Rosbag
    IMU_BAG = rosbag.Bag(data_path)
    # CSV File
    imuFile = open(sample_path + "/imu.csv", mode="w")
    imu_writer = csv.writer(imuFile, delimiter=',', quoting=csv.QUOTE_MINIMAL)
    imu_writer.writerow(['time [s]', 'ax [m s-2]', 'ay [m s-2]', 'az [m s-2]', 'p [rad/s]', 'q [rad/s]', 'r [rad/s]'])
    # Data loading
    flag = False
    init_time = -1
    for topic, msg, t in IMU_BAG.read_messages(topics=['data']):
        if not flag:
            flag = True
            init_time = t.to_nsec()*1e-9
        imu_writer.writerow([t.to_nsec()*1e-9 - init_time,msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z,msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z])
    IMU_BAG.close()    

# ----------------------------------------------------------------------------
# Function for DVS rosbag data loading (full resolution)
# ----------------------------------------------------------------------------
def extract_dvs_data_full_res(data_path):
    print(" .. Loading the DVS rosbag")
    # Rosbag
    DVS_BAG = rosbag.Bag(data_path)
    # Variables for DVS data
    dvs_time = []
    dvs_x = []
    dvs_y = []
    dvs_p = []
    # Data loading
    flag = False
    init_time = -1
    previousTime = 0
    currentTime = 0
    index = 0
    printProgressBar(index, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
    for topic, msg, t in DVS_BAG.read_messages(topics=['data']):
        if not flag:
            flag = True
            init_time = msg.events[0].ts.to_nsec()*1e-9
        printProgressBar(index + 1, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
        for i in range(len(msg.events)):
            currentTime = msg.events[i].ts.to_nsec()*1e-9 - init_time
            dvs_time.append(currentTime)
            dvs_x.append(DIMX - 1 - int(msg.events[i].x))
            dvs_y.append(DIMY - 1 - int(msg.events[i].y))
            if msg.events[i].polarity == True:
                dvs_p.append(1)
            else:
                dvs_p.append(0)
        index = index + 1
    DVS_BAG.close()
    return dvs_time, dvs_x, dvs_y, dvs_p

# ----------------------------------------------------------------------------
# Function for DVS rosbag data loading (half resolution)
# ----------------------------------------------------------------------------
def extract_dvs_data_half_res(data_path):
    print(" .. Loading the DVS rosbag")
    # Rosbag
    DVS_BAG = rosbag.Bag(data_path)
    # Variables for DVS data
    dvs_time = []
    dvs_x = []
    dvs_y = []
    dvs_p = []
    # Data loading
    flag = False
    init_time = -1
    previousTime = 0
    currentTime = 0
    index = 0
    printProgressBar(index, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
    for topic, msg, t in DVS_BAG.read_messages(topics=['data']):
        if not flag:
            flag = True
            init_time = msg.events[0].ts.to_nsec()*1e-9
        printProgressBar(index + 1, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
        for i in range(len(msg.events)):
            currentTime = msg.events[i].ts.to_nsec()*1e-9 - init_time
            current_x = DIMX - 1 - int(msg.events[i].x)
            current_y = DIMY - 1 - int(msg.events[i].y)
            if current_x%2 == 1 and current_y%2 == 1:
                dvs_time.append(currentTime)
                dvs_x.append(int((current_x-1)/2))
                dvs_y.append(int((current_y-1)/2))
                if msg.events[i].polarity == True:
                    dvs_p.append(1)
                else:
                    dvs_p.append(0)
        index = index + 1
    DVS_BAG.close()
    return dvs_time, dvs_x, dvs_y, dvs_p

# ----------------------------------------------------------------------------
# Function for DVS rosbag data loading (mask, half resolution)
# ----------------------------------------------------------------------------
def extract_dvs_data_mask_half_res(data_path):
    print(" .. Loading the DVS rosbag")
    # Rosbag
    DVS_BAG = rosbag.Bag(data_path)
    # Variables for DVS data
    dvs_time = []
    dvs_x = []
    dvs_y = []
    dvs_p = []
    # Mask parameters
    inner_radius = 15
    outer_radius = 45
    # Data loading
    flag = False
    init_time = -1
    previousTime = 0
    currentTime = 0
    index = 0
    printProgressBar(index, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
    for topic, msg, t in DVS_BAG.read_messages(topics=['data']):
        if not flag:
            flag = True
            init_time = msg.events[0].ts.to_nsec()*1e-9
        printProgressBar(index + 1, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
        for i in range(len(msg.events)):
            currentTime = msg.events[i].ts.to_nsec()*1e-9 - init_time
            current_x = DIMX - 1 - int(msg.events[i].x)
            current_y = DIMY - 1 - int(msg.events[i].y)
            if current_x%2 == 1 and current_y%2 == 1:
                current_x = int((current_x-1)/2)
                current_y = int((current_y-1)/2)
                if (current_x - DIMX//4)**2 + (current_y - DIMY//4)**2 >= inner_radius**2 and (current_x - DIMX//4)**2 + (current_y - DIMY//4)**2 < outer_radius**2:
                    dvs_time.append(currentTime)
                    dvs_x.append(current_x)
                    dvs_y.append(current_y)
                    if msg.events[i].polarity == True:
                        dvs_p.append(1)
                    else:
                        dvs_p.append(0)
        index = index + 1
    DVS_BAG.close()
    return dvs_time, dvs_x, dvs_y, dvs_p

# ----------------------------------------------------------------------------
# Function for DVS rosbag data conversion into .mp4 video (full resolution)
# ----------------------------------------------------------------------------
def dvs2vid_full_res(data_path, sample_path, fps):
    print(" .. Converting the DVS dataset into video (" + str(fps) + " fps)")
    # Rosbag
    DVS_BAG = rosbag.Bag(data_path)
    # DVS Frames
    dvs_frames = []
    dvs_myFrame = 128*np.ones((DIMY,DIMX,3), 'uint8')
    # Data loading
    flag = False
    init_time = -1
    previousTime = 0
    currentTime = 0
    index = 0
    printProgressBar(index, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
    for topic, msg, t in DVS_BAG.read_messages(topics=['/cam0/events']):
        if not flag:
            flag = True
            init_time = msg.events[0].ts.to_nsec()*1e-9
        printProgressBar(index + 1, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
        for i in range(len(msg.events)):
            currentTime = msg.events[i].ts.to_nsec()*1e-9 - init_time
            if currentTime - previousTime < 1/fps:
                if msg.events[i].polarity == True:
                    dvs_myFrame[DIMY - 1 - int(msg.events[i].y),DIMX - 1 - int(msg.events[i].x),2] = 255
                    dvs_myFrame[DIMY - 1 - int(msg.events[i].y),DIMX - 1 - int(msg.events[i].x),0] = 0
                else:
                    dvs_myFrame[DIMY - 1 - int(msg.events[i].y),DIMX - 1 - int(msg.events[i].x),0] = 255
                    dvs_myFrame[DIMY - 1 - int(msg.events[i].y),DIMX - 1 - int(msg.events[i].x),2] = 0
            else:
                dvs_frames.append(dvs_myFrame)
                previousTime = currentTime
                dvs_myFrame = 128*np.zeros((DIMY,DIMX,3), 'uint8')
                if currentTime - previousTime < 1/fps:
                    if msg.events[i].polarity == True:
                        dvs_myFrame[DIMY - 1 - int(msg.events[i].y),DIMX - 1 - int(msg.events[i].x),2] = 255
                        dvs_myFrame[DIMY - 1 - int(msg.events[i].y),DIMX - 1 - int(msg.events[i].x),0] = 0
                    else:
                        dvs_myFrame[DIMY - 1 - int(msg.events[i].y),DIMX - 1 - int(msg.events[i].x),0] = 255
                        dvs_myFrame[DIMY - 1 - int(msg.events[i].y),DIMX - 1 - int(msg.events[i].x),2] = 0
        index = index + 1
    dvs_frames.append(dvs_myFrame)
    DVS_BAG.close()

    f = plt.figure(frameon=False, figsize=(3, 3*DIMY/DIMX), dpi=300)
    canvas_width, canvas_height = f.canvas.get_width_height()

    # Open an ffmpeg process
    outf = sample_path + "dvs_full_res.mp4"
    cmdstring = ('ffmpeg', 
        '-y', '-r', '%d' % fps,
        '-s', '%dx%d' % (canvas_width, canvas_height), # size of image string
        '-pix_fmt', 'argb', # format
        '-f', 'rawvideo',  '-i', '-', # tell ffmpeg to expect raw video from the pipe
        '-vcodec', 'mpeg4', outf) # output encoding
    p = subprocess.Popen(cmdstring, stdin=subprocess.PIPE)

    for i in range(len(dvs_frames)):

        plt.imshow(Image.fromarray(dvs_frames[i]), aspect='auto')

        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        
        f.canvas.draw()
        string = f.canvas.tostring_argb()
        p.stdin.write(string)

        plt.cla()
        plt.clf()

    p.communicate()

# ----------------------------------------------------------------------------
# Function for DVS rosbag data conversion into .mp4 video (half resolution)
# ----------------------------------------------------------------------------
def dvs2vid_half_res(data_path, sample_path, fps):
    print(" .. Converting the DVS dataset into video (" + str(fps) + " fps)")
    # Rosbag
    DVS_BAG = rosbag.Bag(data_path)
    # DVS Frames
    dvs_frames = []
    dvs_myFrame = 128*np.ones((DIMY//2,DIMX//2,3), 'uint8')
    # Data loading
    flag = False
    init_time = -1
    previousTime = 0
    currentTime = 0
    index = 0
    printProgressBar(index, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
    for topic, msg, t in DVS_BAG.read_messages(topics=['data']):
        if not flag:
            flag = True
            init_time = msg.events[0].ts.to_nsec()*1e-9
        printProgressBar(index + 1, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
        for i in range(len(msg.events)):
            currentTime = msg.events[i].ts.to_nsec()*1e-9 - init_time
            current_x = DIMX - 1 - int(msg.events[i].x)
            current_y = DIMY - 1 - int(msg.events[i].y)
            if current_x%2 == 1 and current_y%2 == 1:
                if currentTime - previousTime < 1/fps:
                    if msg.events[i].polarity == True:
                        dvs_myFrame[int((current_y-1)/2),int((current_x-1)/2),2] = 255
                        dvs_myFrame[int((current_y-1)/2),int((current_x-1)/2),0] = 0
                    else:
                        dvs_myFrame[int((current_y-1)/2),int((current_x-1)/2),0] = 255
                        dvs_myFrame[int((current_y-1)/2),int((current_x-1)/2),2] = 0
                else:
                    dvs_frames.append(dvs_myFrame)
                    previousTime = currentTime
                    dvs_myFrame = 128*np.zeros((DIMY//2,DIMX//2,3), 'uint8')
                    if currentTime - previousTime < 1/fps:
                        if msg.events[i].polarity == True:
                            dvs_myFrame[int((current_y-1)/2),int((current_x-1)/2),2] = 255
                            dvs_myFrame[int((current_y-1)/2),int((current_x-1)/2),0] = 0
                        else:
                            dvs_myFrame[int((current_y-1)/2),int((current_x-1)/2),0] = 255
                            dvs_myFrame[int((current_y-1)/2),int((current_x-1)/2),2] = 0
        index = index + 1
    dvs_frames.append(dvs_myFrame)
    DVS_BAG.close()

    f = plt.figure(frameon=False, figsize=(3, 3*(DIMY//2)/(DIMX//2)), dpi=300)
    canvas_width, canvas_height = f.canvas.get_width_height()

    # Open an ffmpeg process
    outf = sample_path + "/dvs_half_res.mp4"
    cmdstring = ('ffmpeg', 
        '-y', '-r', '%d' % fps,
        '-s', '%dx%d' % (canvas_width, canvas_height), # size of image string
        '-pix_fmt', 'argb', # format
        '-f', 'rawvideo',  '-i', '-', # tell ffmpeg to expect raw video from the pipe
        '-vcodec', 'mpeg4', outf) # output encoding
    p = subprocess.Popen(cmdstring, stdin=subprocess.PIPE)

    for i in range(len(dvs_frames)):

        plt.imshow(Image.fromarray(dvs_frames[i]), aspect='auto')

        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        
        f.canvas.draw()
        string = f.canvas.tostring_argb()
        p.stdin.write(string)

        plt.cla()
        plt.clf()

    p.communicate()

# ----------------------------------------------------------------------------
# Function for DVS rosbag data conversion into .mp4 video (mask + drop res)
# ----------------------------------------------------------------------------
def dvs2vid_mask_half_res(data_path, sample_path, fps):
    print(" .. Converting the DVS dataset into video (" + str(fps) + " fps)")
    # Rosbag
    DVS_BAG = rosbag.Bag(data_path)
    # DVS Frames
    dvs_frames = []
    dvs_myFrame = 128*np.ones((DIMY//2,DIMX//2,3), 'uint8')
    # Mask parameters
    inner_radius = 15
    outer_radius = 45
    # Data loading
    flag = False
    init_time = -1
    previousTime = 0
    currentTime = 0
    index = 0
    printProgressBar(index, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
    for topic, msg, t in DVS_BAG.read_messages(topics=['/dvs/events']):
        if not flag:
            flag = True
            init_time = msg.events[0].ts.to_nsec()*1e-9
        printProgressBar(index + 1, DVS_BAG.get_message_count(), prefix = 'Progress:', suffix = 'Complete', length = 50)
        for i in range(len(msg.events)):
            currentTime = msg.events[i].ts.to_nsec()*1e-9 - init_time
            current_x = DIMX - 1 - int(msg.events[i].x)
            current_y = DIMY - 1 - int(msg.events[i].y)
            if current_x%2 == 1 and current_y%2 == 1:
                current_x = int((current_x-1)/2)
                current_y = int((current_y-1)/2)
                if (current_x - DIMX//4)**2 + (current_y - DIMY//4)**2 >= inner_radius**2 and (current_x - DIMX//4)**2 + (current_y - DIMY//4)**2 < outer_radius**2:
                    if currentTime - previousTime < 1/fps:
                        if msg.events[i].polarity == True:
                            dvs_myFrame[current_y,current_x,2] = 255
                            dvs_myFrame[current_y,current_x,0] = 0
                        else:
                            dvs_myFrame[current_y,current_x,0] = 255
                            dvs_myFrame[current_y,current_x,2] = 0
                    else:
                        dvs_frames.append(dvs_myFrame)
                        previousTime = currentTime
                        dvs_myFrame = 128*np.zeros((DIMY//2,DIMX//2,3), 'uint8')
                        if currentTime - previousTime < 1/fps:
                            if msg.events[i].polarity == True:
                                dvs_myFrame[current_y,current_x,2] = 255
                                dvs_myFrame[current_y,current_x,0] = 0
                            else:
                                dvs_myFrame[current_y,current_x,0] = 255
                                dvs_myFrame[current_y,current_x,2] = 0
        index = index + 1
    dvs_frames.append(dvs_myFrame)
    DVS_BAG.close()

    f = plt.figure(frameon=False, figsize=(3, 3*(DIMY//2)/(DIMX//2)), dpi=300)
    canvas_width, canvas_height = f.canvas.get_width_height()

    # Open an ffmpeg process
    outf = sample_path + "/dvs_mask_half_res.mp4"
    cmdstring = ('ffmpeg', 
        '-y', '-r', '%d' % fps,
        '-s', '%dx%d' % (canvas_width, canvas_height), # size of image string
        '-pix_fmt', 'argb', # format
        '-f', 'rawvideo',  '-i', '-', # tell ffmpeg to expect raw video from the pipe
        '-vcodec', 'mpeg4', outf) # output encoding
    p = subprocess.Popen(cmdstring, stdin=subprocess.PIPE)

    for i in range(len(dvs_frames)):

        plt.imshow(Image.fromarray(dvs_frames[i]), aspect='auto')
        
        plt.gca().axes.get_xaxis().set_visible(False)
        plt.gca().axes.get_yaxis().set_visible(False)
        
        f.canvas.draw()
        string = f.canvas.tostring_argb()
        p.stdin.write(string)

        plt.cla()
        plt.clf()

    p.communicate()

# ----------------------------------------------------------------------------
# Progress bar 
# ----------------------------------------------------------------------------
def printProgressBar(iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    Source: https://stackoverflow.com/questions/3173320/text-progress-bar-in-the-console
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()
