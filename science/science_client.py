#!/usr/bin/env python
from Science import CustomScience.srv
from __future__ import print_function
import time


def main():
    rospy.init_node('science_client')
    while not rospy.is_shutdown():
        rospy.wait_for_service('Science')
        try:
            ob=rospy.ServiceProxy('Science',CustomScience)
            req=CustomScience()
            req.motor=input('Enter operation number')# req =1 ratchet up down , req =2 servo ,req =3 rotatebase ,req =4 syringe
            req.dir=input('Enter direction')#request nostep:0,first operation:1,second operation:-1
            status=ob(req)
            print(status)
        except rospy.ServiceException as e:
            print('Operation failed due to exception : %s'%e)
            time.sleep(2)

if __name__ == 'main':
    main()
