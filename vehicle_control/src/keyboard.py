#!/usr/bin/env python

import rospy
from project_drive_msgs.msg import CarParams # import the custom message
import curses
forward = 0;
left = 0;


if __name__ == '__main__':
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(1)
    rospy.init_node('keyboard_control', anonymous=True)
    pub = rospy.Publisher('drive_parameters', CarParams, queue_size=10)

    stdscr.refresh()

    key = ''
    while key != ord('q') and not rospy.is_shutdown():
            key = stdscr.getch()
            stdscr.refresh()

            if key == curses.KEY_UP:
                forward += 10        
            elif key == curses.KEY_DOWN:
                forward -= 10 
            if key == curses.KEY_LEFT:
                left += 0.1 
            elif key == curses.KEY_RIGHT:
                left -= 0.1
            elif key == curses.KEY_DC:
                # this key will center the steer and throttle
                forward = 0
                left = 0

            if forward > 100: forward = 100
            if forward < -100: forward = -100
            if left > 90: left = 90
            if left < -90: left = -90

            msg = CarParams()
            msg.velocity = forward
            msg.angle = left
            pub.publish(msg)

    curses.endwin()
