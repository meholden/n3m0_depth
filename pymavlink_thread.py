from threading import Thread
from pymavlink import mavutil
import time

class pymavlinkThread(Thread):
# set up a class for each message
# save whatever is of interest here
    class Heartbeat(object):
        def __init__(self,time=0,msg="no data"):
            self.time=time
            self.msg=msg
##    class Depth(object):
##        def __init__(self,time=0,ft=999):
##            self.time=time
##            self.ft=ft
##    class Temperature(object):
##        def __init__(self,time=0,degC=999):
##            self.time=time
##            self.degC=degC
##
# init all the things.            
    def __init__(self, port, baud):
        Thread.__init__(self)
        self.port = port
        self.baud = baud
        # add classes
        self.heartbeat = self.Heartbeat()
##        self.speed = self.Speed() 
##        self.depth = self.Depth()
##        self.temperature = self.Temperature()
        self.kill = False
##        
##    def parseGPS(self,nmeastr):
##        if nmeastr.find('VHW') >=0:
##            msg = pynmea2.parse(nmeastr,check=False)
##            self.speed.kt = msg.data[4]
##            self.speed.time=time.time()
##        if nmeastr.find('DBT') >= 0:
##            msg = pynmea2.parse(nmeastr,check=False)
##            self.depth.ft = msg.data[0]
##            self.depth.time = time.time()
##            ##print self.depth.time
##        # $YXMTW,22.8,C*
##        if nmeastr.find('MTW') >=0:
##            msg = pynmea2.parse(nmeastr,check=False)
##            self.temperature.degC = msg.data[0]
##            self.temperature.time=time.time()
##            
##	def sendStatus(self,message):
##		# Send a message for QGC to read out loud
##		#  Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY
##		self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,message.encode())
##
	
    
    def run(self):
        self.master = mavutil.mavlink_connection(self.port, baud=self.baud, source_system=1)

        
        while (not self.kill):
            try:
                msg = self.master.recv_match()
                if not msg:
                    continue
                #print("msg type: \t%s\n" % msg.get_type());
                
                if msg.get_type() == 'HEARTBEAT':
                    print("\n\n*****Got message: %s*****" % msg.get_type())
                    print("Message: %s" % msg)
                    print("\nAs dictionary: %s" % msg.to_dict())
                    # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
                    print("\nSystem status: %s" % msg.system_status)
                

                if msg.get_type() == 'GPS_RAW_INT':
                    GPS = msg.to_dict();
                    print("Lat is %s" % (float(GPS["lat"])/1e7))
                    print("Lon is %s" % (float(GPS["lon"])/1e7))
        
            except Exception as e:
                print("error!=======")
                print("===")
                print(str(e))
                print("=====")
        print("end of mavlink!")

 
if __name__ == '__main__':

    try:
        port = "/dev/ttyS0"

        mavlink = pymavlinkThread(port, 57600)
        print("Starting up...")
        mavlink.start()

        while (True):
            if (old_speed != nmea.speed.time):
                print("SPEED, Depth, temp: " + str(nmea.speed.kt) + ", " + str(nmea.depth.ft)
                      + ", " + str(nmea.temperature.degC)
                      + " " + str(nmea.speed.time-old_speed) + " " + str(nmea.depth.time-old_depth))
                old_speed = nmea.speed.time
            if (old_depth != nmea.depth.time):
                print("speed, DEPTH, temp: " + str(nmea.speed.kt) + ", " + str(nmea.depth.ft)
                      + ", " + str(nmea.temperature.degC)
                      + " " + str(nmea.speed.time-old_speed) + " " + str(nmea.depth.time-old_depth))
                old_depth = nmea.depth.time
                ii=ii+1
            #time.sleep(0.5)
                
    except KeyboardInterrupt:
        mavlink.kill = True
        print("Goodbye!")
        




"""
From: Example of connecting to an autopilot via serial communication using pymavlink
"""

# Import mavutil
from pymavlink import mavutil

import time

# Create the connection
# Need to provide the serial port and baudrate
#master = mavutil.mavlink_connection("/dev/ttyS0", baud=57600)
master = mavutil.mavlink_connection("/dev/ttyS0", baud=57600, source_system=1)

# Restart the ArduSub board !
##master.reboot_autopilot()

### Get all information !
##while True:
##    try:
##        print(master.recv_match().to_dict())
##    except:
##        pass
##    time.sleep(0.1)
##

# Wait for packets to come in and decode
# if location is updated, check for new NMEA, if new NMEA then update log file
while True:
    msg = master.recv_match()
    if not msg:
        continue
    print("msg type: \t%s\n" % msg.get_type());
    
##    if msg.get_type() == 'HEARTBEAT':
##        print("\n\n*****Got message: %s*****" % msg.get_type())
##        print("Message: %s" % msg)
##        print("\nAs dictionary: %s" % msg.to_dict())
##        # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
##        print("\nSystem status: %s" % msg.system_status)
    

    if msg.get_type() == 'GPS_RAW_INT':
        GPS = msg.to_dict();
        print("Lat is %s" % (float(GPS["lat"])/1e7))
        print("Lon is %s" % (float(GPS["lon"])/1e7))
        

print "All done"

