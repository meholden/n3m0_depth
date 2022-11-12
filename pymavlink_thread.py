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
            self.lat = 0
            self.lon = 0
            self.COG = 0
            self.SOG = 0
            self.fix = 0
            
    class RC_CHANNELS(object):
        def __init__(self,time=0):
            self.time=time
            self.chan1 = 0
            self.chan2 = 0
            self.chan3 = 0
            self.chan4 = 0
            self.chan5 = 0
            self.chan6 = 0
            self.chan7 = 0
            self.chan8 = 0

    class BATTERY_STATUS(object):
        def __init__(self,time=0):
            self.time=time
            self.batt1V = 0
            
    class ATTITUDE(object):
        def __init__(self,time=0):
            self.time=time
            self.pitch_deg = 0 
            self.roll_deg = 0 
            self.yaw_deg = 0 
            self.pitchrate = 0
            self.yawrate = 0
            self.rollrate = 0
            
# init all the things.            
    def __init__(self, port, baud):
        Thread.__init__(self)
        self.port = port
        self.baud = baud
        # add classes from messages
        self.heartbeat = self.Heartbeat()
        self.gps = self.GPS()
        self.rc = self.RC_CHANNELS()
        self.battery = self.BATTERY_STATUS()
        self.attitude = self.ATTITUDE()
        
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
        
    def sendStatus(self,message):
            # Send a message for QGC to read out loud
            #  Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,message.encode())

	
    
    def run(self):
        self.master = mavutil.mavlink_connection(self.port, baud=self.baud, source_system=1)

    # This code should get the following from the mavlink messages:
    # battery voltage
    # heading
    # COG
    # SOG
    # RC inputs from Tx (for logging on/off)

    # Also need for logging:
    # nmea depth
    # nmea temp
    # tide lookup
    
        
        while (not self.kill):
            try:
                msg = self.master.recv_match()
                if not msg:
                    continue
##                print("msg type: \t%s\n" % msg.get_type());
                
                if msg.get_type() == 'HEARTBEAT':
                    self.heartbeat.msg = msg.to_dict()
                    self.heartbeat.time = time.time()
##                    print("\n\n*****Got message: %s*****" % msg.get_type())
##                    print("Message: %s" % msg)
##                    print("\nAs dictionary: %s" % msg.to_dict())
##                    # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
##                    print("\nSystem status: %s" % msg.system_status)
                
                if msg.get_type() == 'BATTERY_STATUS':
                    m2d = msg.to_dict()
                    self.battery.time = time.time()
                    self.battery.batt1V = (m2d["voltages"][0]/1e3)
                    #print("\nBatt: %f" % (m2d["voltages"][0]/1e3))
                    
##                if msg.get_type() == 'POWER_STATUS':
##                    print("\n\n*****Got message: %s*****" % msg.get_type())
##                    print("Message: %s" % msg)
##                    print("\nAs dictionary: %s" % msg.to_dict())

                if msg.get_type() == 'RC_CHANNELS':
                    m2d = msg.to_dict()
                    self.rc.time = time.time()
                    self.rc.chan1 = m2d["chan1_raw"]
                    self.rc.chan2 = m2d["chan2_raw"]
                    self.rc.chan3 = m2d["chan3_raw"]
                    self.rc.chan4 = m2d["chan4_raw"]
                    self.rc.chan5 = m2d["chan5_raw"]
                    self.rc.chan6 = m2d["chan6_raw"]
                    self.rc.chan7 = m2d["chan7_raw"]
                    self.rc.chan8 = m2d["chan8_raw"]
##                    print("\n\n*****Got message: %s*****" % msg.get_type())
##                    print("Message: %s" % msg)
##                    print("\nAs dictionary: %s" % msg.to_dict())

                if msg.get_type() == 'ATTITUDE':
                    m2d = msg.to_dict()
                    self.attitude.time = time.time()
                    self.attitude.pitch_deg = float(m2d["pitch"])*57.2958
                    self.attitude.roll_deg = float(m2d["roll"])*57.2958
                    self.attitude.yaw_deg = float(m2d["yaw"])*57.2958
                    self.attitude.pitchrate = float(m2d["pitchspeed"])*57.2958
                    self.attitude.rollrate = float(m2d["rollspeed"])*57.2958
                    self.attitude.yawrate = float(m2d["yawspeed"])*57.2958
##                    print("\n\n*****Got message: %s*****" % msg.get_type())
##                    print("Message: %s" % msg)
##                    print("\nAs dictionary: %s" % msg.to_dict())

                if msg.get_type() == 'GPS_RAW_INT':
                    self.gps.GPS = msg.to_dict()
                    self.gps.time = time.time()
                    self.gps.lat = float(self.gps.GPS["lat"])/1e7 # deg
                    self.gps.lon = float(self.gps.GPS["lon"])/1e7 # deg
                    self.gps.COG = float(self.gps.GPS["cog"])/1e2 # deg
                    self.gps.SOG = float(self.gps.GPS["vel"])/1e2 # m/s
                    self.gps.fix = int(self.gps.GPS["fix_type"]) # 2=2D, 3=3D, 4=DGPS, see https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE
##                    print("Lat is %s" % (float(GPS["lat"])/1e7))
##                    print("Lon is %s" % (float(GPS["lon"])/1e7))
        
            except Exception as e:
                print("error!=======")
                print("===")
                print(str(e))
                print("=====")
        print("\nend of mavlink!\n")

 
if __name__ == '__main__':
# Test code
    try:
        port = "/dev/ttyS0"
        
        old_time = time.time()
        old_time2 = time.time()
        print_time = float(time.time())
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
                print("COG = %f" % mavlink.gps.COG)
                print("RC5 = %f" % mavlink.rc.chan5)
                print("Batt = %f" % mavlink.battery.batt1V)
                old_time2=mavlink.gps.time
                  
##            if (old_time != mavlink.heartbeat.time):
##                print("\n dt=%f" % (float(mavlink.heartbeat.time)-float(old_time)))
##                print("dt=%f" % (dt))
##                old_time = mavlink.heartbeat.time
##                ii=ii+1

            if (t1>print_time):
                messag = ("========GPS fix is: %f" % mavlink.gps.fix)
                print(messag)
                mavlink.sendStatus(messag)
                print_time = t1+5
                
            time.sleep(0.1) # this seems important so thread can have time to run.
                
    except KeyboardInterrupt:
        mavlink.kill = True
        print("Goodbye!")
        

