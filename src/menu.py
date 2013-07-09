#!/usr/bin/env python
'''menu.py - menu operations for Nomad1
'''

import roslib; roslib.load_manifest('nomad1')
import rospy
from nomad1.msg import IMU, MotorStatus, MotorCommand
import string
import math
import Tkinter as tk
import ttk

roll = 0
pitch = 0
yaw = 0
yaw_target = 0

encoder_l = 0.0
encoder_r = 0.0
speed_cmd_l = 0.0
speed_cmd_r = 0.0
speed_act_l = 0.0
speed_act_r = 0.0
out_l = 0.0
out_r = 0.0


def set_motor_speed(speed_cmd_l, speed_cmd_r):
    pub.publish(MotorCommand(speed_cmd_l=speed_cmd_l, speed_cmd_r=speed_cmd_r))


def update_motor_speed():
    global yaw_target

    if yaw_target is not None:
        if yaw < (yaw_target - 2):
            spd_stpt_l = 30
            spd_stpt_r = -30
        elif yaw > (yaw_target + 2):
            spd_stpt_l = -30
            spd_stpt_r = 30
        else:
            spd_stpt_l = 0
            spd_stpt_r = 0
            yaw_target = None
        
        if (spd_stpt_l != speed_cmd_l) or (spd_stpt_r != speed_cmd_r):
            set_motor_speed(spd_stpt_l, spd_stpt_r)


def update_imu_status(data):
    global roll
    global pitch
    global yaw

    roll = data.roll
    pitch = data.pitch
    yaw = data.yaw
    #rospy.loginfo('update_imu_status')

    
def update_motor_status(data):
    global encoder_l
    global encoder_r
    global speed_cmd_l
    global speed_cmd_r
    global speed_act_l
    global speed_act_r
    global out_l
    global out_r

    encoder_l = data.encoder_l
    #rospy.loginfo(str(encoder_l))
    encoder_r = data.encoder_r
    speed_cmd_l = data.speed_cmd_l
    speed_cmd_r = data.speed_cmd_r
    speed_act_l = data.speed_act_l
    speed_act_r = data.speed_act_r
    out_l = data.out_l
    out_r = data.out_r
    #rospy.loginfo('update motor status')
    update_motor_speed()


def init_subscriptions():
    rospy.init_node('menu', anonymous=True)
    rospy.Subscriber('imu_data', IMU, update_imu_status)
    rospy.Subscriber('motor_data', MotorStatus, update_motor_status)


def init_publisher():
    global pub
    pub = rospy.Publisher('motor_command', MotorCommand)



class MenuFrame(ttk.Frame):

    def face_north(self):
        global yaw_target
        print "face North"
        yaw_target = 0

    def face_south(self):
        global yaw_target
        print "face South"
        yaw_target = 180

    def face_east(self):
        global yaw_target
        print "face East"
        yaw_target = 90

    def face_west(self):
        global yaw_target
        print "face West"
        yaw_target = -90

    def face_angle(self):
        print "face " + self.angle.get()
        yaw_target = int(self.angle.get())

    def set_speed(self):
        global yaw_target
        print "set speed " + self.speed_l.get() + ", " + self.speed_r.get()
        yaw_target = None
        set_motor_speed(int(self.speed_l.get()), int(self.speed_r.get()))

    def stop(self):
        global yaw_target
        print "STOP"
        yaw_target = None
        set_motor_speed(0, 0)
        

    def __init__(self, parent):
        ttk.Frame.__init__(self, parent)
        self.parent = parent
        self.parent.title("Nomad1 Menu")
        self.style = ttk.Style()
        self.style.theme_use("default")
        self.grid(column=0, row=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        self['padding'] = (5, 5)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        n_button = ttk.Button(self, text="N", command=self.face_north)
        n_button.grid(row=1, column=1)

        w_button = ttk.Button(self, text="W", command=self.face_west)
        w_button.grid(row=2, column=0)

        e_button = ttk.Button(self, text="E", command=self.face_east)
        e_button.grid(row=2, column=2)

        s_button = ttk.Button(self, text="S", command=self.face_south)
        s_button.grid(row=3, column=1)

        angle_label = ttk.Label(self, text="Angle:")
        angle_label.grid(row=4, column=0)

        self.angle = tk.StringVar()
        cmd_field = ttk.Entry(self, width=4, textvariable=self.angle)
        cmd_field.grid(row=4, column=1)

        face_button = ttk.Button(self, text="Face", command=self.face_angle)
        face_button.grid(row=4, column=2)

        speed_l_label = ttk.Label(self, text="Left Speed:")
        speed_l_label.grid(row=5, column=0)

        self.speed_l = tk.StringVar()
        cmd_field = ttk.Entry(self, width=4, textvariable=self.speed_l)
        cmd_field.grid(row=5, column=1)

        speed_r_label = ttk.Label(self, text="Right Speed:")
        speed_r_label.grid(row=6, column=0)

        self.speed_r = tk.StringVar()
        cmd_field = ttk.Entry(self, width=4, textvariable=self.speed_r)
        cmd_field.grid(row=6, column=1)

        set_speed_button = ttk.Button(self, text="Set", command=self.set_speed)
        set_speed_button.grid(row=6, column=2)

        stop_button = ttk.Button(self, text="STOP", command=self.stop)
        stop_button.grid(row=7, column=0, columnspan=3)

        for child in self.winfo_children():
            child.grid_configure(padx=5, pady=5, sticky=(tk.N, tk.S, tk.E, tk.W))


def main():
    root = tk.Tk()
    menu = MenuFrame(root)
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    w = 250
    h = screen_height
    x = screen_width - w
    y = 0
    #root.geometry('%dx%d+%d+%d' % (w, h, x, y))
    init_subscriptions()
    init_publisher()
    root.mainloop()


if __name__ == '__main__':
    main()
