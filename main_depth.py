
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

# TODO: wait for GPS fix before finding location
if (True):  
    # print("fix = " + str(vehicle.gps_0.fix_type) + " Lat=" + str(vehicle.location.global_relative_frame.lat) + " Lon=" + str(vehicle.location.global_relative_frame.lon))

     
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
time_to_geojson = 10 # seconds


while not (nmea.kill or mavlink.kill):
   try:
      if (old_time != nmea.depth.time): # got new nmea data, save it
         old_time = nmea.depth.time
         print("temp = %s" % nmea.temperature.degC)
         print("\n Gdt=%f" % (float(nmea.depth.time) - float(mavlink.gps.time)))
         print("COG = %f" % mavlink.gps.COG)
         print("RC5 = %f" % mavlink.rc.chan5)
         print("Batt = %f" % mavlink.battery.batt1V)
         old_time2=mavlink.gps.time
      
         tide = 0.0 # lookup next

         if (mavlink.rc.chan5 > 1500): # log to file           
              # (lat,lon), datetime, battery, heading, SOG, Temp, Depth, STW, depth time
              myGJ.gjlist.append(makegeojson.geothing([mavlink.gps.lon,mavlink.gps.lat],
                               [time.strftime("%Y-%m-%d %H:%M:%S "),
                                mavlink.battery.batt1V,
                                0.0,  # compass heading todo!
                                mavlink.gps.COG,
                                mavlink.gps.SOG,
                                nmea.temperature.degC,
                                nmea.depth.ft,
                                nmea.speed.kt,
                                nmea.depth.time,
                                tide]))
              print("Logged DEPTH:" + str(nmea.depth.ft)
              + "\tTIDE:" + str('%.2f' % tide)
              + "\tTEMP:" + str(nmea.temperature.degC)
              + "\tLOG:" + str(time_to_geojson-dtime))
         else:
              print("No Log DEPTH:" + str(nmea.depth.ft)
              + "\tTIDE:" + str('%.2f' % tide)
              + "\tTEMP:" + str(nmea.temperature.degC)
              + "\tLOG:" + str(time_to_geojson-dtime))
      
   except Exception as e:
      old_time = nmea.depth.time
      print(e)




   time.sleep(0.1)

	# If new nmea depth data
               # if channel 5 > 1500

		# log depth with most recent location and other telemetry

		# reset nmea depth time

	# if time to commit and push data
	
		# commit and push data
