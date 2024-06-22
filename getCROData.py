# This code was written to get the real-time data from  Tektronix
#Model TBS1072C CRO With  Input Pulse  Settings Horizontal Scale Setting: 4μs, Record Length: 2000 points through USB and store in CSV format in PC
import pyvisa as visa
import numpy as np
from struct import unpack

rm = visa.ResourceManager() 

scope = rm.open_resource('USB0::0x0699::0x03C4::C010503::INSTR') # Connecting via USB 
 
# Setting source as Channel 1
scope.write('DATA:SOU CH1') 
scope.write('DATA:WIDTH 1') 
scope.write('DATA:ENC RPB')

# Getting axis info
ymult = float(scope.query('WFMPRE:YMULT?')) # y-axis least count
yzero = float(scope.query('WFMPRE:YZERO?')) # y-axis zero error
yoff = float(scope.query('WFMPRE:YOFF?')) # y-axis offset
xincr = float(scope.query('WFMPRE:XINCR?')) # x-axis least count   WFMInpre:XZEro?
xzero = float(scope.query('WFMPRE:XZEro?'))
# Reading Binary Data from instrument
scope.write('CURVE?')
data = scope.read_raw() # Reading binary data
# print(data)
headerlen = 2 + int(chr(data[1])) # Finding header length

header = data[:headerlen] # Separating header 

# header_unpacked= np.array(unpack('%sB' % len(header),header))

headerless_wave = data[headerlen:-1] # Separating data
# print(headerless_wave)

# Converting to Binary to ASCII
headerless_wave = np.array(unpack('%sB' % len(headerless_wave),headerless_wave))

scaled_wave = (headerless_wave- yoff ) * ymult + yzero
Time = np.arange(0, xincr *( len(scaled_wave)-1), xincr)+xzero
wave = list(scaled_wave)
time = list(Time)

