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
    class GPS(object):
        def __init__(self,time=0,GPS=[]):
            self.time=time
            self.GPS = GPS
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
        self.gps = self.GPS() 
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
                    self.heartbeat.msg = msg.to_dict()
                    self.heartbeat.time = time.time()
##                    print("\n\n*****Got message: %s*****" % msg.get_type())
##                    print("Message: %s" % msg)
##                    print("\nAs dictionary: %s" % msg.to_dict())
##                    # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
##                    print("\nSystem status: %s" % msg.system_status)
                

                if msg.get_type() == 'GPS_RAW_INT':
                    self.gps.GPS = msg.to_dict()
                    self.gps.time = time.time()
##                    print("Lat is %s" % (float(GPS["lat"])/1e7))
##                    print("Lon is %s" % (float(GPS["lon"])/1e7))
        
            except Exception as e:
                print("error!=======")
                print("===")
                print(str(e))
                print("=====")
        print("\nend of mavlink!\n")

 
if __name__ == '__main__':

    try:
        port = "/dev/ttyS0"
        
        old_time = time.time()
        old_time2 = time.time()
        t0=float(time.time())
        ii=0

        mavlink = pymavlinkThread(port, 57600)
        print("Starting up...")
        mavlink.start()

        while (True):
            t1 = float(time.time())
            dt = t1-t0
            t0=t1
            if (old_time2 != mavlink.gps.time):
##                print("\nAs dictionary: %s" % mavlink.gps.GPS)
##                print("\n t=%s " % mavlink.gps.time )
##                print("\n ot=%s " % old_time)
                print("\n Gdt=%f" % (float(mavlink.gps.time)-float(old_time2)))
##                print("dt=%f" % (dt))
                old_time2=mavlink.gps.time
                  
            if (old_time != mavlink.heartbeat.time):
                print("\n dt=%f" % (float(mavlink.heartbeat.time)-float(old_time)))
                print("dt=%f" % (dt))
                old_time = mavlink.heartbeat.time
                ii=ii+1
            time.sleep(0.1) # this seems important so thread can have time to run.
                
    except KeyboardInterrupt:
        mavlink.kill = True
        print("Goodbye!")
        

