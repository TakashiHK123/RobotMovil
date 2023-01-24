#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from prueba_client.srv import addition, additionResponse

def add_two_ints_client(x, y): #este es el handle
    rospy.wait_for_service('add_two_ints') #esta linea espera e identifica el server al que queremos conectarnos
    add_two_ints = rospy.ServiceProxy('add_two_ints', addition) #una vez que lo encontramos nos conectamos a el en esta linea
    resp1 = add_two_ints(x, y) #aqui no importa como le llamemos a las variables. En el server SI importa
    print(resp1.result) 
    

if __name__ == "__main__":
    x = int(sys.argv[1])
    y = int(sys.argv[2])
    add_two_ints_client(x,y)

