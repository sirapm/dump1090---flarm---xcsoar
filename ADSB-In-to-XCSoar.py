"""
Version 0.7.2
This python code is reading the output from dump1090-mutability.service port 3003 if setup as per default.
The Python code will start track all aircrafts within the range of MaxTrackDistance (in meter).
Every second:
    * it will read gps information, send a copy of the gps data to XCSoar and remember current position, altitude etc.
    * calculate flarm information for tracked aircrafts
    * drop information about aircrafts that have not been seen for MaxTimeDiff seconds
    * generate both PFLAA and PFLAU data and send to XCSoar

For this to work, you need:
    * rtl-sdr dongle with functioning dump1090-mutability.service
    * usb serial GPS

It is recommended to create a udevd rule with an alias as the serial
port can change depending on order Linux is discovering usb serial ports.
/etc/udev/rules.d/gps.rules
KERNEL=="ttyACM*", ATTRS{product}=="*GPS*Receiver*", SYMLINK+="ttyACM_GPS", RUN+="/bin/stty -F /dev/ttyACM_GPS 38400 raw -echo"

configure XCSoar to listen on device udp port 2000 generic

Changes:
23-Dec-2023: Modifying Distance to include two distance values. This way we can see if an aircraft is getting closer or not.
If getting closer, alert, if not, no alert.
If lower or higher than MaxHightDiff meter of own aircraft, no alert.

"""

#from dataclasses import dataclass
from dataclasses import dataclass, astuple, asdict, fields
import socket
import re
import datetime
import geopy
from geopy import distance
from geographiclib.geodesic import Geodesic
import math
import serial
import time

#Add IACO codes you do not want to show up. Typically your own transponder.
IgnoreMyID=["C80897","ZZZZZ"]
SericalPort='/dev/ttyACM_GPS'

#Default Values
"AIRCRAFTS = GPS MSG data"
AIRCRAFTS={}
"DISTANCE {'aircraft id':Meters} "
DISTANCE={}
LASTSEEN={}
"FLARMDATA=  All the flarm fields"
FLARMDATA={}
FLARMLIST=[]
GPSTime="2000-01-01 00:00:00"
Testdata=False
LastCurrentTime=""

#Specify what ip address and port dump1090-mutability is running on
HOST = "192.168.151.111"
PORT = 30003
#Specify udp port where XCSoar is listening on (generic)
XCHost="127.0.0.1"
XCPort=2000

#Set your limitations for Maximum distance of tracking and alarm levels based on distance
#Pwr is powered aircrafts
MaxTrackDistance=30000
Alert1Distance=300
Alert2Distance=150
Alert3Distance=75
Alert1DistancePwr=900
Alert2DistancePwr=500
Alert3DistancePwr=300

#Define how long to wait for an ADSB message of an aircraft till it is considered gone.
MaxTimeDiff=60 #Seconds
MaxHightDiff=100 #Meters

#sp=serial.Serial(SericalPort,timeout=0.1)
sp=""
adsb=""
xcsoaru2000 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
GPSConnected=False
adsbConnected=False
debug=True
previous_second=""


@dataclass()
class MSG:
    Type: str = ""
    Transmission: int = 0
    SessionID: str = ""
    AircraftID: str = ""
    IACO: str = ""
    FlightID: str = ""
    Date: str = ""
    Time: str = ""
    LogDate: str = ""
    LogTime: str = ""
    Callsign: str = ""
    Altitude: int = 0
    GroundSpeed: int = 0
    Track: int = 0
    Latitude: float = 0.0
    Longitude: float = 0.0
    VerticalRate: float = 0.0
    Squawk: str = ""
    Alert: str = ""
    Emergency: str = ""
    SPI: str = ""
    IsOnGround: str = ""

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not isinstance(value, field.type):
                if value == "" and field.type == float:
                    value=0.0
                elif value == "" and field.type == int:
                    value=0
                setattr(self, field.name, field.type(value))

@dataclass
class PFLAU:
    RX: int = 0
    TX: int = 1
    GPS: int = 2
    Power: int = 1
    AlarmLevel: int = 0
    RelativeBearing: int = 0
    AlarmType: int = 0
    RelativeVertical: int = 0
    RelativeDistance: int = 0
    ID: str = ""

    def __str__(self):
        #tmp=f'$PFLAU,{self.RX},{self.TX},{self.GPS},{self.Power},{self.AlarmLevel},{self.RelativeBearing},{self.AlarmType},{self.RelativeVertical},{self.RelativeDistance},{self.ID}*'
        tmp=f'$PFLAU'
        for field in fields(self):
            tmp += f',{getattr(self, field.name)}'
        tmp+="*"
        return tmp+"%02x\r\n" % checksum(tmp)


@dataclass()
class PFLAA:
    AlarmLevel: int = 0
    RelativeNorth: int = 0
    RelativeEast: int = 0
    RelativeVertical: int = 0
    IDType: int = 0
    ID: str = ""
    Track: int = 0
    TurnRate: int = 0
    GroundSpeed: int = 0
    ClimbRate: float = 0.0
    AcftType: str = "A"
    NoTrack: int = 0
    Source: int = 1
    RSSI: int = -50

    def __str__(self):
        #tmp=f'$PFLAA,{self.AlarmLevelc},{self.RelativeNorth},{self.RelativeEast},{self.RelativeVertical},{self.IDType},{self.ID},{self.Track},{self.TurnRate},{self.GroundSpeed},{self.ClimbRate},{self.AcftType},{self.NoTrack},{self.Source},{self.RSSI}*'
        tmp=f'$PFLAA'
        for field in fields(self):
            tmp += f',{getattr(self, field.name)}'
        tmp+="*"
        return tmp+"%02x\r\n" % checksum(tmp)

@dataclass
class GPS:
    Long: float = 0.0
    Lat: float = 0.0
    Alt: int = 0
    Sats: int = 0
    Fix: bool = False

@dataclass
class Details:
    ID: str
    Long: float
    Lat: float
    Track: int = 0






def checksum(sentence):
    """ Remove leading $ """
    sentence = sentence.lstrip('$')
    nmeadata,cksum = re.split('\*', sentence)
    #print nmeadata
    calc_cksum = 0
    for s in nmeadata:
      calc_cksum ^= ord(s)
    return calc_cksum

def MyPos(gga):
    global My
    gaaparts=gga.split(',')
    #print(gaaparts)
    if str(gaaparts[6])!='0':
        My.Fix=True
    else:
        My.Fix=False
        return 0
    pos=str(gaaparts[2]).find('.')
    if pos > -1:
        My.Lat=float(float(gaaparts[2][:pos-2])+(float(gaaparts[2][pos-2:])/60))
    else:
        My.Lat=0.0
    #MyLong=float(gaaparts[2])
    if gaaparts[3]!='N':
        My.Lat*=-1
    pos=str(gaaparts[4]).find('.')
    if pos > -1:
        My.Long=float(float(gaaparts[4][:pos-2])+(float(gaaparts[4][pos-2:])/60))
    else:
        My.Long=0.0
    if gaaparts[5]!='E':
        My.Long*=-1
    My.Alt=int(round(float(gaaparts[9])))
    if gaaparts[7]!='':
        My.Sats=int(gaaparts[7])

def getGPSTimestamp(data):
    global GPSTime
    parts=data.split(',')
    if parts[1] != "" and parts[9] != "":
        time=str(parts[1])
        timesec=time[0:2]+":"+time[2:4]+":"+time[4:6]
        date=str(parts[9])
        datesec="20"+date[4:6]+"-"+date[2:4]+"-"+date[0:2]
        GPSTime=datesec+" "+timesec

def GetGPSData():
    #Get GPS data from serial GPS
    global GPSConnected
    global sp
    if GPSConnected != True:
        print("GPS not connected, try to connect")
        try:
            sp=serial.Serial(SericalPort,timeout=0.1)
        except:
            sp="Fail"
        print(sp)
        if sp != "Fail":
            GPSConnected=True
    if GPSConnected:
        try:
            raw=sp.readall()
        except:
            return()
        gpsdata=raw.decode("utf-8")
        parts=gpsdata.split('\r\n')
        for part in parts:
            #If we have gps signal, of any kind, send it to xcsoar
            if len(part) >10:
                xcsoaru2000.sendto((str(part)+'\r\n').encode(),(XCHost, XCPort))
            if part[:6] == '$GPGGA':
                MyPos(part)
            elif part[:6] == '$GPRMC':
                getGPSTimestamp(part)

def LastSeen(iaco,date,time):
    #Keeping a list of all planes received with the last seen timestamp
    lastdate=date.split('/')
    lasttime=time.split('.')[0].split(':')
    #timestamp=datetime.datetime(int(lastdate[0]),int(lastdate[1]),int(lastdate[2]),int(lasttime[0]),int(lasttime[1]),int(lasttime[2]))
    #ADSB time seem to be in loal timezone. A bit probelmatic so using current gps time.
    timestamp=datetime.datetime.fromisoformat(str(GPSTime))
    LASTSEEN.update({iaco:timestamp})



def ProcessADSBData(row):
    #This function is splitting up a ADSB message and store the info in global AIRCRAFTS for each plane. Ignoring my own ADSB messages.
    global AIRCRAFTS
    global LASTSEEN
    entries=row.split(',')
    #Make sure we have enough elements and icaco length is 6 char.
    if (len(entries) > 16) and (len(entries[4]) == 6) and entries[0] == 'MSG' and not entries[4] in IgnoreMyID:
        msg=MSG(*entries)
        iaco=msg.IACO
        #if not msg.IACO in IgnoreMyID:
        tmpMSG=MSG()
        try:
            tmpMSG=AIRCRAFTS[msg.IACO]
        except:
            tmpMSG=MSG()
            #Set a default/initial Last seen timestamp when a new plane is discovered.
            timestamp=datetime.datetime.fromisoformat(str(GPSTime))
            LASTSEEN.update({iaco:timestamp})
        #When assigning the data from entries, they all seem to become strings and not numerical where they should be.
        tmpMSG=MSG(*entries[0:9]+list(astuple(tmpMSG))[9:])
        #print(tmpMSG)
        #Have seen some planes with more information than normal so if there, get the info.
        #if entries[0] == 'MSG':
        if entries[1] == "1":
            tmpMSG.Callsign=msg.Callsign
        elif entries[1] == "3" or entries[1] == "4":
            #print(msg)
            if msg.Callsign != "":
                 tmpMSG.Callsign=entries[10]
            if msg.Altitude != '' and msg.Altitude != 0:
                 tmpMSG.Altitude=round(int(msg.Altitude)/3)
            if msg.GroundSpeed != '' and msg.GroundSpeed != 0:
                 tmpMSG.GroundSpeed=int(msg.GroundSpeed)
            if msg.Track != '' and msg.Track != 0:
                 tmpMSG.Track=int(msg.Track)
            if msg.Latitude != '' and msg.Latitude != 0.0:
                 #print("============== msg.Latitude: ",end="")
                 #print(msg.Latitude)
                 tmpMSG.Latitude=float(msg.Latitude)
            if msg.Longitude != '' and msg.Longitude != 0.0:
                 tmpMSG.Longitude=float(msg.Longitude)
            if msg.VerticalRate != '' and msg.VerticalRate != 0.0:
                 tmpMSG.VerticalRate=float(msg.VerticalRate)
            if iaco != "" and msg.Date != "" and msg.Time != "":
                 LastSeen(iaco,msg.Date,msg.Time)
        elif entries[1] == "5" or entries[1] == "6" and msg.Altitude != 0:
            tmpMSG.Altitude=round(int(msg.Altitude)/3)
        AIRCRAFTS.update({iaco:tmpMSG})



def GetADSBData():
    global adsbConnected
    global adsb
    #Establish connection the dump1090
    if adsbConnected != True:
        #print("Not connected, try to connect")
        try:
           adsb = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
           result=adsb.connect((HOST, PORT))
        except:
           result="Fail"
        if result != "Fail":
            adsbConnected=True
    if adsbConnected:
        data=""
        try:
            raw = adsb.recv(4096)
            data=raw.decode("utf-8")
        except:
            data=""
        darray=""
        if len(data) > 75 or Testdata:
            darray=data.splitlines()
            #for each section call process line
            section=""
            for section in darray:
                #print(section)
                if section != '':
                    ProcessADSBData(section)

def CalcDistance():
    global DISTANCE
    #GetMyLocation()
    for plane in list(AIRCRAFTS.keys()):
        if float(AIRCRAFTS[plane].Latitude) != 0.0 and float(AIRCRAFTS[plane].Longitude) != 0.0:
            Lat=float(AIRCRAFTS[plane].Latitude)
            Long=float(AIRCRAFTS[plane].Longitude)
            dist=int(str(distance.distance((My.Lat,My.Long),(Lat,Long)).meters).split(".")[0])
            try:
                PrevDist=DISTANCE[plane]
            except:
                PrevDist=[dist,0]
            PrevDist[1]=PrevDist[0]
            PrevDist[0]=dist
            DISTANCE.update({plane:PrevDist})

def FlarmCalc():
    global CloseFlarm
    global pflau
    global FLARMDATA
    global DISTANCE
    global LASTSEEN
    global AIRCRAFTS
    FLARMDATA={}
    pflau=PFLAU()
    for plane in list(AIRCRAFTS.keys()):
        #check age of plane. If old, drop it
        last=""
        try:
            last=LASTSEEN[plane]
        except:
            last=""
        timediff=0
        #print(last)
        if last != "":
            #WARNING: is adsb date in utc or not
            #I see some strange date/time. For now, will change over to use GPS time rather than ADSB time.
            timediff=int(str((datetime.datetime.fromisoformat(str(GPSTime))-datetime.datetime.fromisoformat(str(last))).total_seconds()).split('.')[0])
        if timediff > MaxTimeDiff or last == "":
            if debug:
                print(str(plane)+" Way old or missing, Remove it from dictionaries..... timediff: ",end="")
                print(timediff)
            if plane in LASTSEEN:
                tmp=LASTSEEN.pop(plane)
            if plane in AIRCRAFTS:
                tmp=AIRCRAFTS.pop(plane)
            if plane in DISTANCE:
                tmp=DISTANCE.pop(plane)
            if plane in FLARMDATA:
                tmp=FLARMDATA.pop(plane)
    CloseFlarm=[]
    for plane in list(DISTANCE.keys()):
        if DISTANCE[plane][0] < MaxTrackDistance:
            #print("Time to do some Flarm calcs... " + plane + " Distance: "+str(DISTANCE[plane]))
            pflaa=""
            if plane in FLARMDATA:
                pflaa=FLARMDATA[plane]
            else:
                pflaa=PFLAA()
            pflaa.ID=plane
            pflaa.IDType=1
            pflaa.GroundSpeed=AIRCRAFTS[plane].GroundSpeed
            pflaa.Track=AIRCRAFTS[plane].Track
            if AIRCRAFTS[plane].Callsign != "":
                #pflaa[ID]=AIRCRAFTS[plane][10]
                if "ZKG" in  AIRCRAFTS[plane].Callsign:
                    pflaa.AcftType="1"
                elif "ANZ" in AIRCRAFTS[plane].Callsign or "UAL" in AIRCRAFTS[plane].Callsign or AIRCRAFTS[plane].GroundSpeed > 230:
                    pflaa.AcftType="9"
                elif AIRCRAFTS[plane].Callsign.startswith("ZK"):
                    pflaa.AcftType="8"
                else:
                    pflaa.AcftType="A"
            else:
                pflaa.AcftType="A"
            if pflaa.AcftType=="1":
                Alert3=Alert3Distance
                Alert2=Alert2Distance
                Alert1=Alert1Distance
            else:
                Alert3=Alert3DistancePwr
                Alert2=Alert2DistancePwr
                Alert1=Alert1DistancePwr
            if AIRCRAFTS[plane].Altitude > MaxHightDiff and My.Alt > (AIRCRAFTS[plane].Altitude-MaxHightDiff)  and My.Alt < (AIRCRAFTS[plane].Altitude+MaxHightDiff):
                if DISTANCE[plane][0] < DISTANCE[plane][1]:
                    if int(DISTANCE[plane][0]) < Alert3:
                        pflaa.AlarmLevel=3
                    elif int(DISTANCE[plane][0]) < Alert2:
                        pflaa.AlarmLevel=2
                    elif int(DISTANCE[plane][0]) < Alert1:
                        pflaa.AlarmLevel=1
                    else:
                        pflaa.AlarmLevel=0
                else:
                    if int(DISTANCE[plane][0]) < Alert3:
                        pflaa.AlarmLevel=2
                    else:
                        pflaa.AlarmLevel=0
            else:
                pflaa.AlarmLevel=0

            if AIRCRAFTS[plane].VerticalRate != 0.0:
                #Warning need to find out what the climb rate actually is!
                pflaa.ClimbRate=round(float(AIRCRAFTS[plane].VerticalRate)/3,1)
            else:
                pflaa.ClimbRate=0.0
            if AIRCRAFTS[plane].Latitude != 0.0 and AIRCRAFTS[plane].Longitude != 0.0:
                Lat=float(AIRCRAFTS[plane].Latitude)
                Long=float(AIRCRAFTS[plane].Longitude)
                pflaa.RelativeNorth=int(str(distance.distance((Lat,My.Long),(My.Lat,My.Long)).meters).split(".")[0])
                #print(AIRCRAFTS[plane].Callsign)
                #print(My.Lat)
                #print(Lat)
                if My.Lat > Lat:
                    pflaa.RelativeNorth=pflaa.RelativeNorth*-1
                pflaa.RelativeEast=int(str(distance.distance((My.Lat,Long),(My.Lat,My.Long)).meters).split(".")[0])
                if My.Long > Long:
                    pflaa.RelativeEast = pflaa.RelativeEast * -1
                #pflaa.RelativeNorth=int(str(distance.distance((My.Lat,My.Long),(Lat,My.Long)).meters).split(".")[0])
                #pflaa.RelativeEast=int(str(distance.distance((My.Lat,My.Long),(My.Lat,Long)).meters).split(".")[0])
                #print("Rel north: ",end="")
                #print(pflaa.RelativeNorth)
                #print("Rel EasTh: ",end="")
                #print(pflaa.RelativeEast)
                #Only add planes with coordinations or we have a python crash...
                if AIRCRAFTS[plane].Altitude !="":
                    altdiff=int(AIRCRAFTS[plane].Altitude)-My.Alt
                else:
                    altdiff=0
                pflaa.RelativeVertical=altdiff
                #print("Altitude stuff")
                #print(My.Alt)
                #print(AIRCRAFTS[plane].Altitude)
                #print(altdiff)
                #Should we have alarm level highest with least distance
                if (int(pflaa.AlarmLevel) > int(pflau.AlarmLevel)) or (int(pflaa.AlarmLevel) == int(pflau.AlarmLevel) and int(DISTANCE[plane][0]) < int(pflau.RelativeDistance)):
                    #Right order!
                    result = Geodesic.WGS84.Inverse(My.Lat,My.Long,Lat,Long)
                    bearing = result["azi1"] # in [°] (degrees)
                    #bearing=calc_bearing((Lat,Long), (MyLat,MyLong))
                    pflau.AlarmLevel=pflaa.AlarmLevel
                    pflau.RelativeBearing=int(round(bearing,0))
                    pflau.AlarmType=2
                    pflau.RelativeVertical=altdiff
                    pflau.RelativeDistance=int(DISTANCE[plane][0])
                    pflau.ID=pflaa.ID
                CloseFlarm.append(plane)
                FLARMDATA.update({plane:pflaa})
                pflau.RX=len(CloseFlarm)
                #Debug, check number of elements in PFLAU. should be 10


My=GPS()
pflau=PFLAU()


while True:
    GetADSBData()
    time.sleep(0.001)
    #if debug:
    #print(".",end="")
    CurrentTime=str(datetime.datetime.now())[11:19]
    if CurrentTime != LastCurrentTime:
        LastCurrentTime=CurrentTime
        GetGPSData()
        #print(AIRCRAFTS)
        CalcDistance()
        #print(DISTANCE)
        FlarmCalc()
        xcsoaru2000.sendto((str(pflau)).encode(),(XCHost, XCPort))
        for plane in CloseFlarm:
            xcsoaru2000.sendto(str(FLARMDATA[plane]).encode(),(XCHost, XCPort))
        #sendPFLAA()
        if not adsbConnected:
            xcsoaru2000.sendto(("Waiting for adsb data. No connection."+'\r\n').encode(),(XCHost, XCPort))


