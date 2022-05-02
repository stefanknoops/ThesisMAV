#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 18 11:01:38 2022

@author: stefan
"""
import rosbag as rb


file = 3

#read files
DVSbag = rb.Bag("sample_"+str(file)+".bag")

frequency = 250
rate = 1/frequency

start = False

msgarray = []

NewBag = rb.Bag("sample_"+str(file)+"_freq.bag",mode='w')

for msg in DVSbag:
    
    if msg.topic != '/dvs/events':
        NewBag.write(msg.topic, msg.message, msg.timestamp)
        continue
    
    if not start:
        begintime = msg.timestamp.to_sec()
        start = True
    

    
    for flow in msg.message.events:
        msgarray.append(flow)
        
    if msg.timestamp.to_sec() - begintime >= rate:
        begintime = msg.timestamp.to_sec()
        msg.message.events = msgarray
        msgarray = []
        NewBag.write('/dvs/events', msg.message, msg.timestamp)
    
        
         
