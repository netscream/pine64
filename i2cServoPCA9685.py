#!/usr/bin/env python
'''
        Pine64 - PCA9685 PWM Servo driver
        Derived from the ADAFRUIT raspberry pi driver located at
        https://learn.adafruit.com/micropython-hardware-pca9685-pwm-and-servo-driver/software?view=all

        Free to use according to their license
        @Hlynur Hansen - hlynur@tolvur.net

        NOTE:
                To be able to use the I2C on the pine you need to download
                Enable Pullup from pine64 and download i2ctools, python-smbus

                EnablePullup ->
                http://wiki.pine64.org/images/d/d8/EnableI2cPullup.tar.gz

                $sudo apt-get install python-smbus i2c-tools
        USAGE:
                from i2cServoPCA9685 import PWN_DRIVER as PWN

                def run():
                    a = PWN()
                    a.test_servo()
'''
import smbus
import time
import math


class PWN_DRIVER:

    '''
        name: __init__
        Input: Nothing
        Short: Class init function
        return: Nothing
    '''
    def __init__(self, DRIVER_ADDR=0x40, I2C_CHAN=1, DEBUG=0):
        ''' Here come the constants for this program '''
        self.DEBUG = DEBUG
        self.I2C_CHAN = I2C_CHAN
        self.DRIVER_ADDR = DRIVER_ADDR

        # Set the bus to correct channel
        self.bus = smbus.SMBus(self.I2C_CHAN)

        ''' Constants here are from the adafruit python scripts'''
        # Registers/etc.
        self.MODE1 = 0x00
        self.MODE2 = 0x01
        self.SUBADR1 = 0x02
        self.SUBADR2 = 0x03
        self.SUBADR3 = 0x04
        self.PRESCALE = 0xFE
        self.LED0_ON_L = 0x06
        self.LED0_ON_H = 0x07
        self.LED0_OFF_L = 0x08
        self.LED0_OFF_H = 0x09
        self.ALL_LED_ON_L = 0xFA
        self.ALL_LED_ON_H = 0xFB
        self.ALL_LED_OFF_L = 0xFC
        self.ALL_LED_OFF_H = 0xFD

        # Bits
        self.RESTART = 0x80
        self.SLEEP = 0x10
        self.ALLCALL = 0x01
        self.INVRT = 0x10
        self.OUTDRV = 0x04

        # Servo constants
        self.MAX_DEGREE = 180
        self.MIN_PWR = 150
        self.MAX_PWR = 600

    '''
        name: initialize
        Input:
        Short: Initializes the bus to the driver through I2C_CHAN
        return: Nothing
    '''
    def initialize(self):
        try:
            ''' Software Reset, by setting the led0_low'''
            self.bus.write_byte(self.DRIVER_ADDR, self.LED0_ON_L)

            ''' Set all pwm to 0 '''
            self.bus.write_byte_data(self.DRIVER_ADDR,
                                     self.ALL_LED_ON_L,
                                     0 & 0xFF)
            self.bus.write_byte_data(self.DRIVER_ADDR,
                                     self.ALL_LED_ON_H,
                                     0 >> 8)
            self.bus.write_byte_data(self.DRIVER_ADDR,
                                     self.ALL_LED_OFF_L,
                                     0 & 0xFF)
            self.bus.write_byte_data(self.DRIVER_ADDR,
                                     self.ALL_LED_OFF_H,
                                     0 & 0xFF)

            ''' Setting modes for starter'''
            self.bus.write_byte_data(self.DRIVER_ADDR,
                                     self.MODE2,
                                     self.OUTDRV)
            self.bus.write_byte_data(self.DRIVER_ADDR,
                                     self.MODE1,
                                     self.ALLCALL)
            time.sleep(0.005)
            mode1 = self.bus.read_byte_data(self.DRIVER_ADDR,
                                            self.MODE1)
            mode1 = mode1 & ~self.SLEEP
            self.bus.write_byte_data(self.DRIVER_ADDR,
                                     self.MODE1,
                                     mode1)
            time.sleep(0.005)
        except IOError:
                print("initialize " +
                      " gave errors on self.DRIVER ADDR: " +
                      str(str(self.DRIVER_ADDR)))
        else:
            if self.DEBUG:
                print("Done intializing the driver")

        '''
            We will start by using 60Hz for the servos,
            if you need more then setFreq
        '''
        self.setFreq(60)

    '''
        name: setFreq
        Input: Frequency in Hz
        Short: Function to set the Frequency of the bus driver
        return: Nothing
    '''
    def setFreq(self, freq):
        presc = 25000000.0  # 25mhz
        presc /= 4096.0     # 12-bit PCM9685
        presc /= float(freq)
        presc -= 1.0
        presc = math.floor(presc + 0.5)

        oldm = self.bus.read_byte_data(self.DRIVER_ADDR, self.MODE1)
        newm = (oldm & 0x7F) | 0x10
        try:
            self.bus.write_byte_data(self.DRIVER_ADDR, self.MODE1, newm)
            self.bus.write_byte_data(self.DRIVER_ADDR,
                                     self.PRESCALE,
                                     int(math.floor(presc)))
            self.bus.write_byte_data(self.DRIVER_ADDR, self.MODE1, oldm)
            time.sleep(0.005)
            self.bus.write_byte_data(self.DRIVER_ADDR, self.MODE1, oldm | 0x80)
        except IOError:
                print("setFreq "
                      " gave errors on self.DRIVER ADDR: " +
                      str(self.DRIVER_ADDR) +
                      ". With frequency set to " +
                      str(freq) + "Hz")
        else:
            if self.Debug:
                print("Frequency set to " + str(freq) + "Hz")

    '''
        name: servo
        Input:  Channel register of the servo (0..15)
                Starting position (0)
                Ending position   (150) =
        Short: Function to drive the servo
        return: Nothing
    '''
    def servo(self, channel=0, on=0, off=150):
        try:
                self.bus.write_byte_data(self.DRIVER_ADDR,
                                         self.LED0_ON_L+4*channel,
                                         on & 0xFF)
                self.bus.write_byte_data(self.DRIVER_ADDR,
                                         self.LED0_ON_H+4*channel,
                                         on >> 8)
                self.bus.write_byte_data(self.DRIVER_ADDR,
                                         self.LED0_OFF_L+4*channel,
                                         off & 0xFF)
                self.bus.write_byte_data(self.DRIVER_ADDR,
                                         self.LED0_OFF_H+4*channel,
                                         off >> 8)
        except IOError:
                print("Channel: " + str(channel) +
                      " gave errors on self.DRIVER ADDR: " +
                      str(str(self.DRIVER_ADDR)))

        else:
            if self.Debug:
                print("Servo on channel: " + str(channel) +
                      " set to on: " + str(on) +
                      " and to off: " + str(off))
    '''
        name: turn_degrees
        Input:  Channel register of servo (0..15)
                Degrees to turn (0..180)
        Short: Function to drive the servo in degrees
        return: Nothing
    '''
    def turn_degrees(self, channel=None, deg=None, rad=None):
        ''' 0 is all the way to the left
            90 is straight forward
            190 is all the way to the right'''
        if channel is None or (deg is None and rad is None):
            print("You need to input some channel and degree's")
            return
        SP = self.MAX_PWR - self.MIN_PWR
        if deg is not None:
            turn_degree = (self.MIN_PWR +
                           SP * deg / self.MAX_DEGREE)
        elif rad is not None:
            turn_degree = (self.MIN_PWR +
                           SP * rad / math.radians(self.MAX_DEGREE))

        if self.DEBUG:
            print("Turns " + str(deg) +
                  " degrees" +
                  " That is in power " + str(turn_degree))

        self.servo(channel, 0, turn_degree)

    '''
        name: test_servo
        Input: Nothing
        Short: Function for testing the servo by turning
        middle(90deg), left(0Deg) , middle(90Deg) , right(180Deg), middle(90Deg)
        return: Nothing
    '''
    def test_servo(self):
        for i in range(0, 15):
            self.turn_degrees(i, 90)
            time.sleep(2)
            self.turn_degrees(i, 0)
            time.sleep(2)
            self.turn_degrees(i, 90)
            time.sleep(2)
            self.turn_degrees(i, 180)
            time.sleep(2)
            self.turn_degrees(i, 90)
