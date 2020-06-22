import rospy
import serial
from geometry_msgs.msg import Twist

vmax ="max velocity "
omegamax ="max omega"

class drive():
    def __init__(self):
        self.vel_sub = rospy.Subscriber("cmd_vel",Twist,self.cmd_cb)
        self.rate = rospy.Rate(100);
        self.my_serial = serial.Serial('/dev/tty/ACMO',115200,timeout=1)

    def cmd_cb(self,data):
        self.v = data.linear.x
        self.omega = data.angular.z

        right_wheel =self.v + self.omega
        left_wheel = self.v - self.omega

        if (left_wheel > 0):
            left_speed = self.mymap(left_wheel, 0, VMAX+OMEGAMAX, 0, 63)
        else:
            left_speed = self.mymap(-left_wheel, 0, VMAX+OMEGAMAX, 65, 127)
        
        if (right_wheel > 0):
            right_speed = self.mymap(right_wheel, 0, VMAX+OMEGAMAX, 129, 191)
        else:
            right_speed = self.mymap(-right_wheel, 0, VMAX+OMEGAMAX, 193, 255)
        
        if (self.v == 0 and self.omega == 0):
            right_wheel = 128
            left_wheel = 64

        # first send command for left wheels
        command = str(left_speed)
        command = int(bin(int(command)).replace("0b",""))

        self.my_serial.write(command)

        # right wheels
        command = str(right_speed)
        command = int(bin(int(command)).replace("0b",""))

        self.my_serial.write(command)

        self.rate.sleep()

    def mymap(self,c,a,b,d,e):
        return d + (c-a)*(e-d)/(b-a)

        
if __name__ == "__main__":
    rospy.init_node("motor_controller")
    drive()
    rospy.spin()