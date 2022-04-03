import time
import pyrebase

config = {
  "apiKey": "WbT2JEhLwVVMGMumYi75LqrxQLafkvcS6lh4UTtv",
  "authDomain": "mila-tracker-c92c8.firebaseapp.com",
  "databaseURL": "https://mila-tracker-c92c8-default-rtdb.firebaseio.com",
  "storageBucket": "mila-tracker-c92c8.appspot.com"
}

firebase = pyrebase.initialize_app(config)
db = firebase.database()
print('Send data')
c=0
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
  db.child("mlx90614").child("2-push").push(data)
  print('hello')

  time.sleep(2)