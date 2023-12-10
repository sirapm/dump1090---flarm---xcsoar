"""
INfo to come

For this to work, you need a:
    * rtl-sdr dongle.
    * GPS
it is recommended to create a udevd rule with an alias as the serial
port can change depending on order Linux is discovering usb serial ports.

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

#Add IACO codes you do not want to show up. Typically your own transponder.
IgnoreMyID=["B80897","OGN123", "ZZZZZ"]
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
Testdata=True
LastCurrentTime=""

#Specify what ip address and port dump1090-mutability is running on
HOST = "127.0.0.1"
PORT = 30003
#Specify udp port where XCSoar is listening on (generic)
XCHost="127.0.0.1"
XCPort=2000

#Set your limitations for Maximum distance of tracking and alarm levels based on distance
#Pwr is powered aircrafts
MaxTrackDistance=10000
Alert1Distance=300
Alert2Distance=150
Alert3Distance=75
Alert1DistancePwr=1000
Alert2DistancePwr=500
Alert3DistancePwr=300

#Define how long to wait for an ADSB message of an aircraft till it is considered gone.
MaxTimeDiff=60

sp=serial.Serial(SericalPort,timeout=0.1)
xcsoaru2000 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
adsbConnected=False
debug=True
previous_second=""


@dataclass(kw_only=False)
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
    

@dataclass(kw_only=False)
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
    #print("Reading GPS")
    try:
        raw=sp.readall()
    except:
        return()
    gpsdata=raw.decode("utf-8")
    parts=gpsdata.split('\r\n')
    #print(parts)
    #print('========')
    for part in parts:
        #If we have gps signal, of any kind, send it to xcsoar
        #print(part)
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
    timestamp=datetime.datetime(int(lastdate[0]),int(lastdate[1]),int(lastdate[2]),int(lasttime[0]),int(lasttime[1]),int(lasttime[2]))
    LASTSEEN.update({iaco:timestamp})


def ProcessADSBData(row):
    #This function is splitting up a ADSB message and store the info in global AIRCRAFTS for each plane. Ignoring my own ADSB messages.
    #global tmpMSG
    global AIRCRAFTS
    #global entries
    entries=row.split(',')
    #Make sure we have enough elements and icaco length is 6 char.
    if (len(entries) > 16) and (len(entries[4]) == 6) and entries[0] == 'MSG' and not entries[4] in IgnoreMyID:
        #print(row)
        msg=MSG(*entries)
        #print(msg)
        iaco=msg.IACO
        #if not msg.IACO in IgnoreMyID:
        tmpMSG=MSG()
        try:
            tmpMSG=AIRCRAFTS[msg.IACO]
        except:
            tmpMSG=MSG()
        #When assigning the data from entries, they all seem to become strings and not numerical where they should be.
        tmpMSG=MSG(*entries[0:9]+list(astuple(tmpMSG))[9:])
        #print(tmpMSG)
        #Have seen some planes with more information than normal so if there, get the info.
        #if entries[0] == 'MSG':
        match entries[1]:
            case "1":
                 tmpMSG.Callsign=msg.Callsign
            case "3"|"4":
                if msg.Callsign != "":
                    tmpMSG.Callsign=entries[10]
                if msg.Altitude != '':
                    tmpMSG.Altitude=int(msg.Altitude)
                if msg.GroundSpeed != '':
                    tmpMSG.GroundSpeed=int(msg.GroundSpeed)
                if msg.Track != '':
                    tmpMSG.Track=int(msg.Track)
                if msg.Latitude != '':
                    tmpMSG.Latitude=float(msg.Latitude)
                if msg.Longitude != '':
                    tmpMSG.Longitude=float(msg.Longitude)
                if msg.VerticalRate != '':
                    tmpMSG.VerticalRate=float(msg.VerticalRate)
                if iaco != "" and msg.Date != "" and msg.Time != "":
                    LastSeen(iaco,msg.Date,msg.Time)
            case "5"|"6":
                 tmpMSG.Altitude=msg.Altitude
        AIRCRAFTS.update({iaco:tmpMSG})



def GetADSBData():
    global adsbConnected
    #Establish connection the dump1090
    if adsbConnected != True:
        try:
            result=s.connect((HOST, PORT))
            #adsbConnedted=True
            #print("Waiting for adsb data. No connection.")
        except:
            result="Fail"
        if result != "Fail":
            adsbConnected=True
    if adsbConnected or Testdata:
        data=""
        try:
            raw = s.recv(4096)
            data=raw.decode("utf-8")
        except:
            data=""
        darray=""
        if len(data) > 75 or Testdata:
            darray=data.splitlines()
            #for each section call process line
            if Testdata: #57.830367649297536, 11.797951228906939
                #print("Sending testdata")
                darray=["MSG,3,5,211,C80897,10057,2023/11/29,06:58:00.594,2023/11/29,06:58:51.153,ZKGNC,3700,111,100,57.875303,11.9,-1,,0,0,0,0",
                        "MSG,3,5,211,BA43F5,10057,2023/11/29,06:58:00.594,2023/11/29,06:58:51.153,ANZ1,1400,111,100,57.87326,11.797,5,,0,0,0,0",
                        "MSG,3,5,211,AABBCC,10057,2023/11/29,06:58:00.594,2023/11/29,06:58:51.153,ZKLOM,1400,111,100,57.893,11.798,3.3,,0,0,0,0"]
            #darray=[]
            section=""
            for section in darray:
                #print(section)
                if section != '':
                    ProcessADSBData(section)

def CalcDistance():
    global DISTANCE
    #GetMyLocation()
    for plane in list(AIRCRAFTS.keys()):
        if AIRCRAFTS[plane].Latitude > 0.0 and AIRCRAFTS[plane].Longitude > 0.0 and AIRCRAFTS[plane].Altitude > 0:
            Lat=float(AIRCRAFTS[plane].Latitude)
            Long=float(AIRCRAFTS[plane].Longitude)
            Altitude=int(AIRCRAFTS[plane].Altitude)
            dist=int(str(distance.distance((My.Lat,My.Long),(Lat,Long)).meters).split(".")[0])
            #print("CalcDistance - Plane: "+plane+ " Distance: " + str(dist))
            DISTANCE.update({plane:dist})

def FlarmCalc():
    global CloseFlarm
    global pflau
    global FLARMDATA
    global DISTANCE
    global LASTSEEN
    global AIRCRAFTS
    FLARMDATA={}
    #PFLAU=[0,1,2,1,0,0,0,0,0,""]
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
            #timediff=int(str((datetime.datetime.utcnow()-datetime.datetime.fromisoformat(str(last))).total_seconds()).split('.')[0])
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
        if DISTANCE[plane] < MaxTrackDistance:
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
                match AIRCRAFTS[plane].Callsign[:3]:
                    case "ZKG":
                        pflaa.AcftType="1"
                    case "ANZ"|"JST"|"AWK":
                        pflaa.AcftType="9"
                    case "ZK.":
                        pflaa.AcftType="8"
                    case _:
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
            if int(DISTANCE[plane]) < Alert3:
                pflaa.AlarmLevel=3
            elif int(DISTANCE[plane]) < Alert2:
                pflaa.AlarmLevel=2
            elif int(DISTANCE[plane]) < Alert1:
                pflaa.AlarmLevel=1
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
                pflaa.RelativeEast=int(str(distance.distance((My.Lat,Long),(My.Lat,My.Long)).meters).split(".")[0])
                #Only add planes with coordinations or we have a python crash...
                if AIRCRAFTS[plane].Altitude !="":
                    altdiff=int(AIRCRAFTS[plane].Altitude)-My.Alt
                else:
                    altdiff=0
                pflaa.RelativeVertical=altdiff
                #Should we have alarm level highest with least distance
                if int(pflaa.AlarmLevel) > int(pflau.AlarmLevel) or int(DISTANCE[plane]) < int(pflau.RelativeDistance):
                    #Right order!
                    result = Geodesic.WGS84.Inverse(My.Lat,My.Long,Lat,Long)
                    bearing = result["azi1"] # in [°] (degrees)
                    #bearing=calc_bearing((Lat,Long), (MyLat,MyLong))
                    pflau.AlarmLevel=pflaa.AlarmLevel
                    pflau.RelativeBearing=int(round(bearing,0))
                    pflau.AlarmType=2
                    pflau.RelativeVertical=altdiff
                    pflau.RelativeDistance=int(DISTANCE[plane])
                    pflau.ID=pflaa.ID
                CloseFlarm.append(plane)
                FLARMDATA.update({plane:pflaa})
                pflau.RX=len(CloseFlarm)
                #Debug, check number of elements in PFLAU. should be 10
   
    
My=GPS()
pflau=PFLAU()


while True:
    GetADSBData()
    #if debug:
    #    print(".",end="")
    CurrentTime=str(datetime.datetime.now())[11:19]
    if CurrentTime != LastCurrentTime:
        LastCurrentTime=CurrentTime
        GetGPSData()
        #print('-')
        #print(My)
        #print(AIRCRAFTS)
        CalcDistance()
        #print(DISTANCE)
        FlarmCalc()
        xcsoaru2000.sendto((str(pflau)).encode(),(XCHost, XCPort))
        for plane in CloseFlarm:
            xcsoaru2000.sendto(str(FLARMDATA[plane]).encode(),('127.0.0.1', 2000))
        #sendPFLAA()
        if not adsbConnected:
            xcsoaru2000.sendto(("Waiting for adsb data. No connection."+'\r\n').encode(),(XCHost, XCPort))




