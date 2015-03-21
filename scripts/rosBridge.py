#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import *
import numpy as np
from swarath.msg import *
pub_gps = rospy.Publisher('gps_unity', String, queue_size=10)
pub_imu = rospy.Publisher('imu_unity', String, queue_size=10)
pub_lidar=rospy.Publisher('lidar_unity',lidar_data,queue_size=10)


rospy.init_node('swarath', anonymous=True)
rate = rospy.Rate(10) # 10hz




import string,cgi,time
from os import curdir, sep
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer


dataRecieved = []
detaToBeSent = [0 , 0]


def parseData(data):
    data = data[1:]
    gps = data.split('sense')[0]
    imu = data.split('sense')[1]
    lidar = data.split('sense')[2]
    gpsData = ''
    imuData = ''
    gps = gps.split('q')
    imu = imu.split('q')

    lidar = lidar.split(';')
    lidar = [ x.split('q') for x in lidar ]
    arr=lidar_data()    # You can take LIDAR Data and change it to any format you want




    for data1 in gps:
		gpsData = gpsData + data1 + ',';

    for data2 in imu:
		imuData = imuData + data2 + ',';

    #lidar = lidar.split(';')
    #lidar = [ x.split('q') for x in lidar]


    # NEW
    maxAngle1 = 60
    minAngle1 = -60
    maxAngle2 = 60
    minAngle2 = -60
    angle1LC = 2
    angle2LC = 2
    arr.increment=angle1LC
    arr.range=maxAngle1


    noOfVerticle = int((maxAngle2 - minAngle2)/angle2LC)
    noOfHorizontal = int((maxAngle1 - minAngle1)/angle1LC)
    arr.vertical_readings=noOfVerticle
    arr.horizontal_readings=noOfHorizontal


    lidarArr = []

    for i in range(noOfVerticle):
        lidarArr.append([])
        for j in range(noOfHorizontal):
            lidarArr[i].append(0)

    for entry in lidar:

        try :
            d = (float(entry[0]))
            t1 = int(entry[1])
            t2 = int(entry[2])

            i1 = (t1 - minAngle1)/angle1LC
            i2 = (t2 - minAngle2)/angle2LC

            # print i1, i2

            lidarArr[i1][i2] = d

        except :
            pass





    #print(lidarArr)
    temp=noOfVerticle*noOfHorizontal
    lidarArr1=np.reshape(lidarArr,temp)
    arr.array=lidarArr1
   # print(lidarArr1)

    gpsData = gpsData[:-1]
    imuData = imuData[:-1]
    #dataRecieved = [gps , imu ]
    pub_gps.publish(str(gpsData))
    pub_imu.publish(str(imuData))
    pub_lidar.publish(arr)


    #print str(dataRecieved)

class MyHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        try:


            self.send_response(200)
            self.send_header('Content-type',	'text/html')
            self.end_headers()
            dataRecieved = self.path
            self.wfile.write( str(detaToBeSent[0]) +  " " + str(detaToBeSent[1]))
            parseData(dataRecieved)
            return



        except IOError:
            self.send_error(404,'File Not Found: %s' % self.path)


    def do_POST(self):
        global rootnode
        try:


            ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))

            if ctype == 'multipart/form-data':
                query=cgi.parse_multipart(self.rfile, pdict)
            self.send_response(301)

            self.end_headers()
            upfilecontent = query.get('upfile')
            # print "filecontent", upfilecontent[0]
            self.wfile.write("<HTML>POST OK.<BR><BR>");
            self.wfile.write(upfilecontent[0]);

        except :
            pass

def sendThrottle(t):
	t = str( (t.data))
	print t
	detaToBeSent[0] = t

def sendSteer(s):
	s = str((s.data))
	print s
	detaToBeSent[1] = s

def listener():
	rospy.Subscriber("throttle_unity", String, sendThrottle)
	rospy.Subscriber("steering_unity", String, sendSteer)




def main():
    try:

        server = HTTPServer(('', 8080), MyHandler)
        print 'started httpserver...'
        server.serve_forever()
    except KeyboardInterrupt:
        print '^C received, shutting down server'
        server.socket.close()

print "haha"

if __name__ == '__main__':
	listener()
	main()


while not rospy.is_shutdown():
	publish()







