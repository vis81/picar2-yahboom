#!/usr/bin/python3
import serial
import sys
import time
import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider, TextBox, RadioButtons
import numpy as np

vel2rpm = 0.166667
ticks2rpm = 60 * 20 / 824

plt.ion()

class PidTest():

    cmd_set_pid = b'motor pid %c %f %f %f\n'
    cmd_set_vel = b'motor vel %c %d\n'
    cmd_set_debug = b'motor debug %c %d\n'

    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.speed = 1000
        self.kp = 0.5
        self.kd = 0.0
        self.ki = 2.4
        self.rpm = 0
        self.motor = b'r'
        self.started = False
        self.f = serial.Serial(self.port, self.baud)
        self.lock = threading.Lock()
    
    # defining function to add line plot
    def reset(self, val):
        print('reset')
        with self.lock:
            self.data_time = []
            self.data_setpoint = []
            self.data_actual = []
            self.data_pwm = []
            self.new_data = True

    # defining function to add line plot
    def stop(self, val):
        print('stop')
        self.started = False
        self.f.write(PidTest.cmd_set_vel % (self.motor, 0))
        self.f.write(PidTest.cmd_set_debug % (self.motor, 0))

    # defining function to add line plot
    def start(self, val):
        print('start')
        if not self.started:
            self.reset(None)
        self.started = True
        self.f.write(PidTest.cmd_set_debug % (self.motor, 1))
        self.f.write(PidTest.cmd_set_vel % (self.motor, self.speed))

    def update_speed(self, val):
        print('update_speed %d' % val)
        self.speed = val
        self.freq_slider.label = str(self.speed)
        if self.started:
            self.f.write(PidTest.cmd_set_vel % (self.motor, self.speed))

    def updatePID(self, val):
        if not self.text_box_kp.text:
            self.kp = 0.0
        else:
            self.kp = float(self.text_box_kp.text)
        if not self.text_box_kd.text:
            self.kd = 0.0
        else:
            self.kd = float(self.text_box_kd.text)
        if not self.text_box_ki.text:
            self.ki = 0.0
        else:
            self.ki = float(self.text_box_ki.text)
        print('updatePID %f %f %f' % (self.kp, self.kd, self.ki))
        self.stop(None)
        self.f.write( PidTest.cmd_set_pid % (self.motor, self.kp, self.kd, self.ki))

    def updateMotor(self, label):
        print("Motor " + label)
        self.stop(None)
        self.motor = bytes(label, encoding='utf-8')
    
    def on_launch(self):
        #Set up plot
        self.figure, self.ax = plt.subplots()
        self.line_setpoint, = self.ax.plot([],[], 'o', ls='-', marker='.')
        self.lines_actual, = self.ax.plot([],[], 'o', ls='-', marker='.')
        self.lines_pwm, = self.ax.plot([],[], 'o', ls='-', marker='.')
        self.ax.set_autoscaley_on(True)
        
        self.figure.subplots_adjust(left=0.1, bottom=0.25)

        self.ax.grid()

        axes = plt.axes([0.81, 0.01, 0.09, 0.075])
        self.breset = Button(axes, 'Reset',color="yellow")
        self.breset.on_clicked(self.reset)

        axes = plt.axes([0.71, 0.01, 0.09, 0.075])
        self.bstop = Button(axes, 'Stop',color="yellow")
        self.bstop.on_clicked(self.stop)

        axes = plt.axes([0.61, 0.01, 0.09, 0.075])
        self.bstart = Button(axes, 'Start',color="yellow")
        self.bstart.on_clicked(self.start)

        axes = plt.axes([0.51, 0.01, 0.09, 0.075])
        self.bupdate = Button(axes, 'Update',color="yellow")
        self.bupdate.on_clicked(self.updatePID)

        
        axes = plt.axes([0.41, 0.01, 0.09, 0.075])
        self.text_box_ki = TextBox(axes, 'KI', initial=str(self.ki))
        axes = plt.axes([0.26, 0.01, 0.09, 0.075])
        self.text_box_kd = TextBox(axes, 'KD', initial=str(self.kd))
        axes = plt.axes([0.11, 0.01, 0.09, 0.075])
        self.text_box_kp = TextBox(axes, 'KP', initial=str(self.kp))

        axfreq = self.figure.add_axes([0.25, 0.1, 0.65, 0.03])
        self.freq_slider = Slider(
            ax=axfreq,
            label='Speed',
            valmin=0,
            valmax=3300,
            valinit=self.speed,
        )
        self.freq_slider.on_changed(self.update_speed)
        
        axes = plt.axes([0.01, 0.01, 0.05, 0.075])
        self.radio = RadioButtons(axes, ('r', 'l'))
        self.radio.on_clicked(self.updateMotor)
        ...

    def thread_function(self, name):
        print("thread start")
        for line in self.f:
            #print(line)
            lines = [i for i in line.split()]
            l = len(lines)
            if l < 4 or not b'PID' in lines[l - 5]:
                print("Skip" + str(line))
                continue
            #print(line)
            with self.lock:
                self.rpm = float(lines[l - 2]) * ticks2rpm
                self.data_time.append(int(lines[l - 4]))
                self.data_setpoint.append(float(lines[l - 3]))
                self.data_actual.append(float(lines[l - 2]))
                self.data_pwm.append(float(lines[l - 1]))
                self.new_data = True
                
                    

    def on_running(self):
        #Update data (with the new _and_ the old points)
        self.line_setpoint.set_xdata(self.data_time)
        self.lines_pwm.set_xdata(self.data_time)
        self.lines_actual.set_xdata(self.data_time)        
        self.line_setpoint.set_ydata(self.data_setpoint)
        self.lines_actual.set_ydata(self.data_actual)
        self.lines_pwm.set_ydata(self.data_pwm)
        #Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()

    #Example
    def __call__(self):
        self.on_launch()
        self.reset(None)
        self.update_speed(self.speed)
        self.new_data = False
        self.thread = threading.Thread(target=self.thread_function, args=(1,))
        self.thread.start()
        #with open("/dev/ttyUSB1", 'r+') as f:
        while True:
            with self.lock:
                if self.new_data:
                    self.ax.set_title("%.2f RPM" % self.rpm)
                    self.new_data = False
                    self.on_running()
            #We need to draw *and* flush
            self.figure.canvas.draw()
            self.figure.canvas.flush_events()
            self.figure.canvas.draw_idle()
            time.sleep(0.05)


port = sys.argv[1]
baud = sys.argv[2]

d = PidTest(port, baud)
d()
input("press enter to close")
