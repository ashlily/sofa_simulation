# Echo client program
import socket
import sys
import rospy
from std_msgs.msg import *
from numpy import *
import numpy as np

tp_msg=[]
def ros_callback(msg):
    global tp_msg
    tp_msg=msg.data
        

HOST = '127.0.0.1'    # The remote host
PORT = 50007              # The same port as used by the server
s = None
for res in socket.getaddrinfo(HOST, PORT, socket.AF_UNSPEC, socket.SOCK_STREAM):
    af, socktype, proto, canonname, sa = res
    try:
	s = socket.socket(af, socktype, proto)
    except socket.error, msg:
	s = None
	continue
    try:
	s.connect(sa)
    except socket.error, msg:
	s.close()
	s = None
	continue
    break
if s is None:
    print 'could not open socket'
    sys.exit(1)
try:
    #ros related
    rospy.init_node('sofa_agent', anonymous=True)
    loop_rate = rospy.Rate(50);
    sofa_pub = rospy.Publisher('feature_trajectory_position', Float64MultiArray, queue_size=100)
    rospy.Subscriber('sofa_chatter', Float64MultiArray, ros_callback);
    s.send('Hello, world')
    data_temp = s.recv(16000)
    #global tp_msg

    while not rospy.is_shutdown():
        #s.send('Hello, world')
       
        #get ros msg 
        str_tp_msg_long = str(tp_msg)
        print "str_tp_msg_long is:"
        print str_tp_msg_long
        str_tp_msg = str_tp_msg_long[1:-1]
        print "str_tp_msg is:"
        print str_tp_msg
        print len(str_tp_msg)
        
        #send data
        if len(str_tp_msg) == 0:
            str_tp_msg = "0.001, 0.000, 0.000"
        print len(str_tp_msg)
        print "wait for ros"
        
        s.send(str_tp_msg)


        data = s.recv(16000)
        #print "data is:"
        #print data
        #print "len data is:"
        #print len(data)
 
        #split data
        if len(data) > 0:
            data_split = data.split(",")
            #print "len data_split is:"
            #print len(data_split)
            #print "data_split is:"
            #print data_split
            #conver date to float64
            data_f = np.float64(data_split)
            #print "len data_f is:"
            #print len(data_f)
            #print "data_f is:"
            #print data_f
            #conver list to array
            data_msg = Float64MultiArray()
            data_msg.data = np.array(data_f)
            print "data_msg is:"
            print data_msg
            print "len data_f is:"
            print len(data_msg.data)
            #publish to ros topic
            sofa_pub.publish(data_msg)
        
       
        
        loop_rate.sleep()
        
    s.close()
    #print 'Received', repr(data)
except rospy.ROSInterruptException:
   pass

