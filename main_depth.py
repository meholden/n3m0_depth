
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
# pymavlink listener thread



# geojson utilities
import makegeojson

# Initialize geojson object

# Main loop:=====================


	# If new nmea depth data

		# log depth with most recent location and other telemetry

		# reset nmea depth time

	# if time to commit and push data
	
		# commit and push data
