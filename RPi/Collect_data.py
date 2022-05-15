import time
import pyrebase
import numpy as np
from scipy.optimize import fsolve,root
#import matplotlib.pyplot as plt
import serial, string, time
import re
import csv
from datetime import datetime



'''
Parse the ToA data from UWB listener

'''
def parse_line(line_str):
    #print("line str: " + line_str)
    m = re.match("Pkt (\d+) - ToA (\d+) to [\d]: (-\S+|\S+) mm\r\n",line_str)
    if(m):
        dt = datetime.now()
        # getting the timestamp
        ts = datetime.timestamp(dt)
        print("MMMM",ts," ",m.group(1),m.group(2),m.group(3))
        return [m.group(1),m.group(2),m.group(3)]
    else:
        return None


'''
Parse the IMU data from UWB listener

'''
def parse_IMU(line_str):
    #print("line str: " + line_str)
    m = re.match("FIN, (\S+), (\S+), (\S+), (\S+), (\S+), (\S+), (\S+),",line_str)
    if(m):
        return [m.group(1),m.group(2),m.group(3),m.group(4),m.group(5),m.group(6),m.group(7)]
    else:
        m = re.match("POLL, (\S+), (\S+), (\S+), (\S+), (\S+), (\S+), (\S+),",line_str)
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

firebase = pyrebase.initialize_app(config)
db = firebase.database()
print('Send data')
c=0

output = ""
ser = serial.Serial('/dev/ttyACM0', 115200,timeout=5)
anchor_loc = np.array([[0,0],[2150,0],[2150,5400],[0,5400], [20,30], [30,40]])
num_anchors = anchor_loc.shape[0]

toa_vec = np.zeros(num_anchors)
current_id = 0
current_ranging_id = 0

filename_IMU = "ACC_Mila_TEST.csv"
filename_Location = "Location_Mila_[TEST]_[room:bedroom].csv"
with open(filename_IMU,'w') as csvfile_IMU:
    with open(filename_Location,'w') as csvfile_Location:
        csvwriter_IMU = csv.writer(csvfile_IMU)
        csvwriter_Location = csv.writer(csvfile_Location)
        print('entering loop')
        counter = 0
        while(True):
            output = ser.readline()
            #print("output: " + str(output))
        #     print(str(output,'UTF-8'))
            if(len(output)>5):
                ans = parse_line(str(output,'UTF-8'))
                ans_IMU = parse_IMU(str(output,'UTF-8'))
                if(ans_IMU):
                    print(ans_IMU)
                    print("############")
                    csvwriter_IMU.writerow(ans_IMU)
                
                if(ans):
                    
                    current_id,device_id,toa = ans[0],ans[1],ans[2]
                    print("pkt ",current_id," device ",device_id," toa ",toa)
                    
                    if(current_id==current_ranging_id):
                        try:
                            toa_vec[int(device_id)-1] = float(toa)
                        except:
                            print("skipped")
                    else:
                        print(toa_vec)
                        if(sum(toa_vec!=0)>=3):
                            print('solving')
                            tag_loc = root(toa_solver, [1, 1],args=(anchor_loc,toa_vec),method='lm')
                            csvwriter_Location.writerow(tag_loc.x)
                            print(float(tag_loc.x[0]))
                            print(float(tag_loc.x[1]))
                            
                            '''
                            data = {
                                    "ambient": tag_loc.x[0],
                                    "object": tag_loc.x[1],
                                  }
                            db.child("mlx90614").child("1-set").set(data)
                            db.child("mlx90614").child("4-push").child("MSJDHJRTI").update(data)
                            '''



                        toa_vec = np.zeros(num_anchors)
                        print('device_id: ' + str(device_id))
                        try:
                            toa_vec[int(device_id)-1] = float(toa)
                        except:
                            print(device_id)
                        current_ranging_id = current_id
            counter += 1
            if counter%100 == 0:
                csvfile_Location.flush()
                csvfile_IMU.flush()
        #print("closed my files")
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

