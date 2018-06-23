#!/usr/bin/env python
'''testpub ROS Node'''
# license removed for brevity
import rospy
from testcan.msg import Frame, IpPos
import time

ip_pos = IpPos()
def sendpos(id, pos):
        ip_pos.pos = pos
        ip_pos.id = id
        return ip_pos

def talker():
    '''testpub Publisher'''
    pub = rospy.Publisher('ip_pos', IpPos, queue_size=10)
    rospy.init_node('testpub', anonymous=True)
    # ip_pos = IpPos()
    rate = rospy.Rate(50) # 10hz
    step = 100000
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # ip_pos.pos = step
        # ip_pos.id = 0x07
        pub.publish(sendpos(13,step))
        # time.sleep(0.5)
        # pub.publish(sendpos(13,-100000))
        # time.sleep(0.5)
        # pub.publish(sendpos(17,72000))  
        # time.sleep(0.001) 
        # pub.publish(sendpos(12,72000))  
        # time.sleep(0.001)
        # pub.publish(sendpos(20,72000))
        # time.sleep(0.001)
        # pub.publish(sendpos(24,72000))
        # time.sleep(0.001)
        # pub.publish(sendpos(11,72000))  
        # time.sleep(0.001) 
        # pub.publish(sendpos(16,72000))  
        # time.sleep(0.001)
        # pub.publish(sendpos(22,0))
        # time.sleep(0.001)
        # pub.publish(sendpos(21,0))
        # time.sleep(0.001)
        # pub.publish(sendpos(17,0))   
        # time.sleep(0.001)
        # pub.publish(sendpos(12,0)) 
        # time.sleep(0.001)
        # pub.publish(sendpos(20,0))
        # time.sleep(0.001)
        # pub.publish(sendpos(24,0))
        # time.sleep(0.001)
        # pub.publish(sendpos(11,0))  
        # time.sleep(0.001) 
        # pub.publish(sendpos(16,0)) 
        # # time.sleep(0.001) 
        # time.sleep(0.001)
        # pub.publish(sendpos(22,-72000))
        # time.sleep(0.001)
        # pub.publish(sendpos(21,-72000))
        # time.sleep(0.001)
        # pub.publish(sendpos(17,-72000))
        # time.sleep(0.001)   
        # pub.publish(sendpos(11,-72000)) 
        # time.sleep(0.001)
        # pub.publish(sendpos(20,-72000))
        # time.sleep(0.001)
        # pub.publish(sendpos(24,-72000))
        # time.sleep(0.001)
        # pub.publish(sendpos(11,-72000))  
        # time.sleep(0.001) 
        # pub.publish(sendpos(16,-72000))  

        print 'seng ip position %s' %step
        rate.sleep()
        step = -step

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
