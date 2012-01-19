#!/usr/bin/env python
"""
control with keyboard
usage:
rosrun explore_beego key_img.py cmd:=/beego/velocity image:=/beego/camera
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('geometry_msgs')
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import wx
import sys
import threading

class TwistPublisher(threading.Thread):
    """ ROS Twist (v,w) Publisher
    http://www.ros.org/doc/api/geometry_msgs/html/msg/Twist.html
    """
    def __init__(self):
        threading.Thread.__init__(self)
        self._cmd = Twist()
    def run(self):
        publisher = rospy.Publisher('/cmd', Twist)
        while not rospy.is_shutdown():
            publisher.publish(self._cmd)
            self._cmd.linear.x *= .9
            self._cmd.angular.z *= .9
            rospy.sleep(.1)

    def cmd_forward(self):
        self._cmd.linear.x = 1 # forward
    def cmd_backward(self):
        self._cmd.linear.x = -1 # backward
    def cmd_left(self):
        self._cmd.angular.z = 1 # turn left
    def cmd_right(self):
        self._cmd.angular.z = -1 # turn right
    def cmd_stop(self):
        self._cmd.linear.x = 0
        self._cmd.angular.z = 0

class ImageViewPanel(wx.Panel):
    """ class ImageViewPanel creates a panel with an image on it, inherits wx.Panel 
    http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html
    """
    def update(self, image):
        if not hasattr(self, 'staticbmp'):
            self.staticbmp = wx.StaticBitmap(self)
            frame = self.GetParent()
            frame.SetSize((image.width, image.height))
        if image.encoding == 'rgba8':
            bmp = wx.BitmapFromBufferRGBA(image.width, image.height, image.data)
            self.staticbmp.SetBitmap(bmp)
        elif image.encoding == 'rgb8':
            bmp = wx.BitmapFromBuffer(image.width, image.height, image.data)
            self.staticbmp.SetBitmap(bmp)

class KeyEventFrame(wx.Frame):
    def __init__(self, parent = None, id = -1, title = __file__):
        wx.Frame.__init__(self, parent, id, title)
        self.mapKeyFn = {}

        self.panel = ImageViewPanel(self)
        self.panel.Bind(wx.EVT_KEY_DOWN, self.OnKeyDown)
        self.panel.SetFocus()
        self.publisher = TwistPublisher()
        self.DoBindKeys()

        self.Centre()
        self.Show()
        rospy.init_node('wxKeyTwist')
        rospy.Subscriber('/image', Image, self.HandleImage)
        self.publisher.start()

    def BindKey(self, key, function):
        self.mapKeyFn[key] = function

    def OnKeyDown(self, event):
        keycode = event.GetKeyCode()
        if keycode in self.mapKeyFn:
            self.mapKeyFn[keycode]()
        event.Skip()

    def HandleImage(self, image):
        # make sure we update in the UI thread
        if self.IsShown():
            wx.CallAfter(self.panel.update, image)
        # http://wiki.wxpython.org/LongRunningTasks

    def DoBindKeys(self):
        """ Bind keys to Twist command
        http://wxpython.org/docs/api/wx.KeyEvent-class.html
        """
        self.BindKey(ord('Z'), self.publisher.cmd_forward)
        self.BindKey(ord('S'), self.publisher.cmd_backward)
        self.BindKey(ord('Q'), self.publisher.cmd_left)
        self.BindKey(ord('D'), self.publisher.cmd_right)
        self.BindKey(wx.WXK_UP,    self.publisher.cmd_forward)
        self.BindKey(wx.WXK_DOWN,  self.publisher.cmd_backward)
        self.BindKey(wx.WXK_LEFT,  self.publisher.cmd_left)
        self.BindKey(wx.WXK_RIGHT, self.publisher.cmd_right)
        self.BindKey(wx.WXK_SPACE, self.publisher.cmd_stop)

def main(argv):
    app = wx.App()
    KeyEventFrame()
    print(__doc__)
    app.MainLoop()
    rospy.signal_shutdown("MainLoop")
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
