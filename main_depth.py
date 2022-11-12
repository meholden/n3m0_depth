
'''  
main code for n3m0 depth code
by Mike Holden 2022
'''

# include all the things:===============
import sys
# account passwords and API tokens come from here
# stored one directory up for easier repo organization and hiding on github
sys.path.append("..") # Adds higher directory to python modules path.
from secrets import gitrepo
import time

# for github commands
import subprocess

# Speed/Depth/Temp via nmea 0183
import nmea_thread

# Start up nmea depth listener thread
try:
   port = "/dev/ttyUSB0"

   nmea = nmea_thread.nmeaThread(port, 4800)
   print("Starting up nmea...")
   nmea.start()
except:
     print("Error doing nmea startup")
     nmea.kill = True


# Start up ardupilot listener thread
import pymavlink_thread

# pymavlink listener thread
try:
     port = "/dev/ttyS0"
     mavlink = pymavlink_thread.pymavlinkThread(port, 57600)
     print("Starting up mavlink...")
     mavlink.start()
except:
     print("Error doing mavlink startup")
     mavlink.kill = True



# geojson utilities
import makegeojson

# Initialize geojson object
import makegeojson

# Setup Tide lookup (requires internet)
# Access online tide data for MLLW reference
import datetime
import gettides


# Wait for GPS fix before finding location for tide
t_now = float(time.time())
t_wait= t_now + 60
t_p = t_now
while (t_now < t_wait):
   t_now = float(time.time())
   
   if (t_now >= t_p):
      messag = ("Waiting for GPS fix: %f" % mavlink.gps.fix)
      print(messag)
      mavlink.sendStatus(messag)
      t_p = t_now+5
      
   if (mavlink.gps.fix >=3):
      t_wait = t_now-1 # stop waiting
      messag = ("Got GPS fix: %f" % mavlink.gps.fix)
      print(messag)
      mavlink.sendStatus(messag)
   time.sleep(0.2)

      
if (mavlink.gps.fix >=3):  
     tides = gettides.tidesfromNOAA()
     stat = tides.nearestStation(mavlink.gps.lat,mavlink.gps.lon)
     #stat = tides.nearestStation(37.99730,-122.09111)
     #tides.setStatName(9415143,"carquinez") # hack here to directly specify station number

     # note that this must be within 24hr of the initialization
     x = datetime.datetime.utcnow()

     tide = tides.lookup(x)

     print ("Right now at " + str(x) + " tide is " + str(tide) + "  meters")
else:
     print ("No GPS, no tides!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")


# geoJson file for storing data
# define variables to be stored here
# uses makegeojson.py
class geoJsonClass:
     def __init__(self):
          self.gjlist=[]
          self.cog = 0
          self.sog = 0
          # (lat,lon), datetime, battery, heading, SOG, Temp, Depth, STW, depth time
          self.dataLabels = makegeojson.geothing(["Latitude(deg)","Longitude(deg)"],
                                                 ["date-time","Battery (Volts)",
                                                  "Heading (deg)",
                                                  "GPS Heading(deg)",
                                                  "GPS Speed(m/s)",
                                                  "Temperature(C)",
                                                  "Depth(ft)",
                                                  "Boatspeed(kt)",
                                                  "Depth Timestamp",
                                                  "Tide station" + str(tides.station) + " (m) "])
myGJ = geoJsonClass();


# Main loop:=====================
old_time = time.time()
old_time2 = old_time
start_time = old_time
time_to_geojson = 10 # seconds
time_to_groundstation = 10
pmsg = "uh oh no data"

## start time used for github updates etc.
start_str = time.strftime("%Y%m%d_%H%M%S")


while not (nmea.kill or mavlink.kill):
   dtime = time.time() - start_time # seconds since go
   try:
      if (old_time != nmea.depth.time): # If new nmea depth data
         old_time = nmea.depth.time
##         print("temp = %s" % nmea.temperature.degC)
##         print("\n Gdt=%f" % (float(nmea.depth.time) - float(mavlink.gps.time)))
##         print("COG = %f" % mavlink.gps.COG)
##         print("RC5 = %f" % mavlink.rc.chan5)
##         print("Batt = %f" % mavlink.battery.batt1V)
##         old_time2=mavlink.gps.time
      
         #tide = 0.0 # lookup next
         x = datetime.datetime.utcnow()
         tide = tides.lookup(x)

         if (mavlink.rc.chan5 > 1500): # log to file based on switch          
              # (lat,lon), datetime, battery, heading, SOG, Temp, Depth, STW, depth time
              myGJ.gjlist.append(makegeojson.geothing([mavlink.gps.lon,mavlink.gps.lat],
                               [time.strftime("%Y-%m-%d %H:%M:%S "),
                                mavlink.battery.batt1V,
                                mavlink.attitude.yaw_deg,  
                                mavlink.gps.COG,
                                mavlink.gps.SOG,
                                nmea.temperature.degC,
                                nmea.depth.ft,
                                nmea.speed.kt,
                                nmea.depth.time,
                                tide]))
              
              pmsg = ("Logged DEPTH:" + str(nmea.depth.ft)
              + "\tTIDE:" + str('%.2f' % tide)
              + "\tTEMP:" + str(nmea.temperature.degC)
              + "\tLOG:" + '%6.1f' % (time_to_geojson-dtime))
              print(pmsg)
         else:
              pmsg = ("No Log DEPTH:" + str(nmea.depth.ft)
              + "\tTIDE:" + str('%.2f' % tide)
              + "\tTEMP:" + str(nmea.temperature.degC)
              + "\tLOG:" + '%6.1f' % (time_to_geojson-dtime))
              print(pmsg)

      if (dtime > time_to_groundstation):
         time_to_groundstation = time_to_groundstation+10
         # status to groundstation
         mavlink.sendStatus(pmsg)
      

      
   except Exception as e:
      old_time = nmea.depth.time
      print(e)


   # if time to commit and push data
   if (dtime > time_to_geojson):
      print("Time to commit---------------")
      time_to_geojson = time_to_geojson+30
      #time_to_geojson = time_to_geojson+120
      if (len(myGJ.gjlist) > 0):
            try:
               print "making file:" + start_str + ".geojson"
               makegeojson.makegeojson("/home/pi/Desktop/dronekitstuff/data_archive/n3m0/" + start_str+".geojson", myGJ.gjlist, myGJ.dataLabels)
            except:
               print "Can't make geojson data file right now"

            # update github repo.  Assumes credentials are stored in gitrepo URL or use (git config --global credential.helper cache)
            try:
               print subprocess.check_output('git add .', shell=True, cwd="/home/pi/Desktop/dronekitstuff/data_archive")
               print subprocess.check_output('git commit -m "auto commit from n3m0"', shell=True, cwd="/home/pi/Desktop/dronekitstuff/data_archive")
               print subprocess.check_output('git push ' + gitrepo, shell=True, cwd="/home/pi/Desktop/dronekitstuff/data_archive")
               # run "git pull" from data_archive directory to sync from web
               # 3 commands above to sync back to web (add/commit/push)
               # username is meholden, oath token for password.
               messag = ("Git push %s" % (start_str + ".geojson"))
               print(messag)
               mavlink.sendStatus(messag)
               
            except Exception as e:
               #print "Can't do github right now"
               messag = ("Git push failed!!! %s" % (start_str + ".geojson"))
               print(messag)
               mavlink.sendStatus(messag)
               print(e)



   time.sleep(0.1) # let background threads go

	

