#!/usr/bin/env python3

import rospy
from prueba_serv.srv import addition, additionResponse
from std_msgs.msg import String

def server_cb(req):
    
    pub.publish("Sumado")
    return additionResponse(req.x + req.y) #en el server es donde manupulamos los datos



if __name__ == "__main__":
    rospy.init_node('add_two_ints_server')
    print('Ready')
    s = rospy.Service('add_two_ints', addition, server_cb)
    pub = rospy.Publisher("topic", String, queue_size=10)
    

    rospy.spin()