import smbus
import RPi.GPIO as gpio
from numpy import array, dot, ones, shape, argmax, eye, zeros, real, sqrt
from numpy.linalg import inv, eig
from scipy.linalg import sqrtm
import time
import sys

# Register addresses
DATA_XOUT_L = 0x00
INT_CTRL_0 = 0x08
INT_CTRL_1 = 0x09

# Local field strength (can be calculated using the WMM model)
F = 51.1928

class mmc5883():

    def __init__(self, device_address, bus=1):
        # Open communications
        self.bus = smbus.SMBus(bus)
        self.address = device_address
        
        # Set sensor output data rate to 200 Hz & fire up
        self.bus.write_byte_data(self.address, INT_CTRL_1, 1)
        
        # Hard & soft iron offsets set by using the calibrate() function
        self.b = zeros([3, 1])
        self.A = eye(3)
        
    def get_mag(self):
        # Begin measurement (this command needs to be sent every time)
        self.bus.write_byte_data(self.address, INT_CTRL_0, 1)
        
        # Read all sensor registers at once in order:
        # mx, mz, my
        data = self.bus.read_i2c_block_data(self.address, DATA_XOUT_L, 6)
        
        values = []
        for lo, hi in zip(data[0::2], data[1::2]):
            # Merge high  bits with low bits & scale to µT
            values.append((((hi << 8) | lo) - 32768) * 0.025)
            
        # Remove offsets
        s = dot(self.A, array(values).reshape(3, 1) - self.b)
        return [s[0, 0], s[1, 0], s[2, 0]]
        
    def calibrate(self):
        print("Calibrating MMC5883 - move sensor around")
        
        # Gather data (make sure to wave the sensor around)
        data = []
        for i in range(600):
            data.append(self.get_mag())
            time.sleep(0.1)
            
            # Print progress bar
            if not i%30:
                print("[{}{}] - {}% \r".format(int(i/30) * "■", (19 - int(i/30)) * ".", int(i / 6)), end = '')
            
        # Fit ellipse to the data (credit to Teslabs Engineering)
        data = array(data).T
        
        D = array([data[0] * data[0],
                   data[1] * data[1],
                   data[2] * data[2],
                   2 * data[1] * data[2],
                   2 * data[0] * data[2],
                   2 * data[0] * data[1],
                   2 * data[0],
                   2 * data[1],
                   2 * data[2],
                   ones(shape(data[0]))])
        
        S = dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]
        
        C = array([[-1,  1,  1,  0,  0,  0],
                   [ 1, -1,  1,  0,  0,  0],
                   [ 1,  1, -1,  0,  0,  0],
                   [ 0,  0,  0, -4,  0,  0],
                   [ 0,  0,  0,  0, -4,  0],
                   [ 0,  0,  0,  0,  0, -4]])
        
        E = dot(inv(C), S_11 - dot(S_12, dot(inv(S_22), S_21)))

        w, v = eig(E)
        
        v_1 = v[:, argmax(w)]
        if v_1[0] < 0: v_1 = -v_1
        v_2 = dot(dot(-inv(S_22), S_21), v_1)

        M = array([[v_1[0], v_1[3], v_1[4]],
                   [v_1[3], v_1[1], v_1[5]],
                   [v_1[4], v_1[5], v_1[2]]])
        n = array([[v_2[0]],
                   [v_2[1]],
                   [v_2[2]]])
        d = v_2[3]
        
        M_1 = inv(M)
        
        # Set the hard and soft offsets
        self.b = -dot(M_1, n)
        self.A = real(F / sqrt(dot(n.T, dot(M_1, n)) - d) * sqrtm(M))
        
