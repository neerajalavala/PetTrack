import time
import pyrebase
import numpy as np
from scipy.optimize import fsolve,root
#import matplotlib.pyplot as plt
import serial, string, time
import re
import csv
import pickle


'''
Parse the ToA data from UWB listener

'''
def parse_line(line_str):
    m = re.match("Pkt (\d+) - ToA (\d+) to [\d]: (-\S+|\S+) mm",line_str)
    if(m):
        return [m.group(1),m.group(2),m.group(3)]
    else:
        return None


'''
Parse the IMU data from UWB listener

'''
def parse_IMU(line_str):
    m = re.match("FIN, (\S+), (\S+), (\S+), (\S+), (\S+), (\S+), (\S+)",line_str)
    if(m):
        return [m.group(1),m.group(2),m.group(3),m.group(4),m.group(5),m.group(6),m.group(7)]
    else:
        m = re.match("POLL, (\S+), (\S+), (\S+), (\S+), (\S+), (\S+), (\S+)",line_str)
        if(m):
            return [m.group(1),m.group(2),m.group(3),m.group(4),m.group(5),m.group(6),m.group(7)]
        else:
            return None

'''
Solve for location based on ToA data
'''
def toa_solver(x,*args):
    anchor_loc = args[0]
    toa_vec = args[1]
    
    a = []
    for i in np.arange(toa_vec.shape[0]):
        if(toa_vec[i]!=0):
            a.append(  np.sqrt((x[0]-anchor_loc[i,0])**2+(x[1]-anchor_loc[i,1])**2)-toa_vec[i]  )
    return a


'''
Set up Firebase messaging
'''

config = {
  "apiKey": "WbT2JEhLwVVMGMumYi75LqrxQLafkvcS6lh4UTtv",
  "authDomain": "mila-tracker-c92c8.firebaseapp.com",
  "databaseURL": "https://mila-tracker-c92c8-default-rtdb.firebaseio.com",
  "storageBucket": "mila-tracker-c92c8.appspot.com"
}
'''
config = {
  "apiKey": "AIzaSyDSYkvrcvM9bVlcu308OKE74UmV153jQsI",
  "authDomain": "mila-tracker-8803.firebaseapp.com",
  "databaseURL": "https://mila-tracker-8803-default-rtdb.firebaseio.com/",
  "storageBucket": "mila-tracker-8803.appspot.com"
}
'''
firebase = pyrebase.initialize_app(config)
db = firebase.database()
print('Send data')
c=0

output = ""
ser = serial.Serial('/dev/ttyACM0', 115200,timeout=5)
# 3.54,3.24
anchor_loc = np.array([[0,0],[8200,0],[8200,5400],[0,5400]])
num_anchors = anchor_loc.shape[0]

toa_vec = np.zeros(num_anchors)
current_id = 0
current_ranging_id = 0
tag_loc_buff = np.zeros((5,2))
buff_pointer = 0
Model = pickle.load(open('new_model.sav', 'rb'))
label = 0
while(True):
    output = ser.readline()
    #print(str(output,'UTF-8'))
    if(len(output)>5):
        ans = parse_line(str(output,'UTF-8'))
        ans_IMU = parse_IMU(str(output,'UTF-8'))
        if(ans_IMU):
            #print("")
            print("@@@@@@@@@")
            print(ans_IMU)
            lst1 = [float(i[0:len(i)-1]) for i in [ans_IMU[1], ans_IMU[2],ans_IMU[3], ans_IMU[4],ans_IMU[5], ans_IMU[6]]]
            lst1[0] = ((lst1[0]+180)/360) if lst1[0]<0 else (lst1[0]/360)
            lst1[1] = ((lst1[1]+180)/360) if lst1[1]<0 else (lst1[1]/360)
            lst1[2] = ((lst1[2]+180)/360) if lst1[2]<0 else (lst1[2]/360)
            lst2=[lst1[0], lst1[1],lst1[2], lst1[3],lst1[4], lst1[5] ]
            print(lst2)
            print(lst1)
            label = Model.predict([lst2])
            print("$$$$")
            print(label)
        if(ans):
            current_id,device_id,toa = ans[0],ans[1],ans[2]
            print("pkt ",current_id," device ",device_id," toa ",toa)
            if(current_id==current_ranging_id):
                toa_vec[int(device_id)-1] = float(toa)-300
            else:
                print(toa_vec)
                if(sum(toa_vec!=0)>=3):
                    print('solving')
                    tag_loc = root(toa_solver, [1, 1],args=(anchor_loc,toa_vec),method='lm')
                    tag_loc_buff[buff_pointer,:] = tag_loc.x
                    buff_pointer = buff_pointer+1
                    if(buff_pointer>=5):
                        buff_pointer = 0
                        tag_loc_avg = np.mean(tag_loc_buff, axis=0)
                        print(float(tag_loc.x[0]))
                        print(float(tag_loc.x[1]))
                        print("$$$$$$$$$$$$$$$$")
                        print(label[0])
                        data = {
                                "ambient": tag_loc.x[0],
                                "object": tag_loc.x[1],
                                "label": int(label[0])
                              }
                        
                        
                        db.child("mlx90614").child("1-set").set(data)
                        db.child("mlx90614").child("7-push").child("MHDRSTA").update(data)




                toa_vec = np.zeros(num_anchors)
                toa_vec[int(device_id)-1] = float(toa)
                current_ranging_id = current_id

'''
while True:
  a = c+20
  b = c+20
  ambientString = "{:.2f}".format(a)
  objectString = "{:.2f}".format(b)

  ambientCelsius = float(a)
  objectCelsius = float(b)
  print("Ambient Temp: {}".format(ambientString))
  print("Object Temp: {}".format(objectString))
  print('####')
  c = c+1
  data = {
    "ambient": ambientCelsius,
    "object": objectCelsius,
  }
  db.child("mlx90614").child("1-set").set(data)
  db.child("mlx90614").child("2-push").update(data)
  print('hello')

  time.sleep(2)
'''
