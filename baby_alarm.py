#!/usr/bin/env python
import pyaudio
import RPi.GPIO as gpio
import signal
from numpy import zeros,linspace,short,fromstring,hstack,transpose,log
from scipy import fft
from time import sleep
from smbus2 import SMBus
from mlx90614 import MLX90614

bus = SMBus(1)
sensor = MLX90614(bus, address=0x5A)

#Volume Sensitivity, 0.05: Extremely Sensitive, may give false alarms
#             0.1: Probably Ideal volume
#             1: Poorly sensitive, will only go off for relatively loud
SENSITIVITY= 0.5
# Alarm frequencies (Hz) to detect (Use audacity to record a wave and then do Analyze->Plot Spectrum)
TONE = 1600
#Bandwidth for detection (i.e., detect frequencies within this margin of error of the TONE)
BANDWIDTH = 600
# Show the most intense frequency detected (useful for configuration)
frequencyoutput=True

# gpio set
gpio.cleanup()
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio.setup(17, gpio.OUT)
servoPIN = 18
gpio.setup(servoPIN, gpio.OUT)
p = gpio.PWM(servoPIN, 50) # GPIO 18 for PWM with 50Hz
p.start(2.5) # Initialization

#Set up audio sampler - 
NUM_SAMPLES = 8192
SAMPLING_RATE = 44100
pa = pyaudio.PyAudio()
_stream = pa.open(format=pyaudio.paInt16,
                  channels=1, rate=SAMPLING_RATE,
                  input=True,
                  frames_per_buffer=NUM_SAMPLES)


blipcount=0
beepcount=0
resetcount=0
clearcount=0
alarm=False

gpio.output(17,gpio.LOW)

def reset_(signum, _ ):
    gpio.output(17,gpio.LOW)
    p.ChangeDutyCycle(5)
    sleep(0.5)
    print("reset")

def lale():
    gpio.output(17,gpio.HIGH)
    signal.signal(signal.SIGALRM, reset_)
    signal.setitimer(signal.ITIMER_REAL, 10, 0)
    p.ChangeDutyCycle(12.5)
    sleep(0.5)

try:
    while True:
        while _stream.get_read_available()< NUM_SAMPLES: sleep(0.01)
        audio_data  = fromstring(_stream.read(
            _stream.get_read_available(), exception_on_overflow=False), dtype=short)[-NUM_SAMPLES:]
        # Each data point is a signed 16 bit number, so we can normalize by dividing 32*1024
        normalized_data = audio_data / 32768.0
        intensity = abs(fft(normalized_data))[:NUM_SAMPLES/2]
        frequencies = linspace(0.0, float(SAMPLING_RATE)/2, num=NUM_SAMPLES/2)
        if frequencyoutput:
            which = intensity[1:].argmax()+1
            # use quadratic interpolation around the max
            if which != len(intensity)-1:
                y0,y1,y2 = log(intensity[which-1:which+2:])
                x1 = (y2 - y0) * .5 / (2 * y1 - y2 - y0)
                # find the frequency and output it
                thefreq = (which+x1)*SAMPLING_RATE/NUM_SAMPLES
            else:
                thefreq = which*SAMPLING_RATE/NUM_SAMPLES
            print "\t\t\t\tfreq=",thefreq
            
        print "Ambient Temperature :", sensor.get_ambient()
        print "Object Temperature :", sensor.get_object_1()

        if max(intensity[(frequencies < TONE+BANDWIDTH) & (frequencies > TONE-BANDWIDTH )]) > max(intensity[(frequencies < TONE-100) & (frequencies > TONE-200)]) + SENSITIVITY:

            lale()
            print("bebe agliyor")
        if sensor.get_object_1() > 30:
            lale()
            print("bebenin atesi cikti")

        sleep(0.01)

except KeyboardInterrupt:
  p.stop()
  gpio.cleanup()