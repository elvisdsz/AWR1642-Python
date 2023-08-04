import serial
import time
import numpy as np
import pyqtgraph as pg
# from pyqtgraph.Qt import QtGui
from PyQt5 import QtWidgets

# Change the configuration file name
#configFileName = 'mmw_pplcount_demo_default.cfg'
configFileName = 'profile_2023_08_02T11_35_33_607.cfg'
CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0;


import os
script_dir = os.path.dirname(__file__)
configFileName = os.path.join(script_dir, configFileName)


# ------------------------------------------------------------------

# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    
    # Raspberry pi
    #CLIport = serial.Serial('/dev/ttyACM0', 115200)
    #Dataport = serial.Serial('/dev/ttyACM1', 921600)
    
    # Windows
    CLIport = serial.Serial('COM13', 115200)
    Dataport = serial.Serial('COM12', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        CLIport.write((i+'\n').encode())
        print(i)
        time.sleep(0.01)
        
    return CLIport, Dataport

# ------------------------------------------------------------------

# Function to parse the data inside the configuration file
def parseConfigFile(configFileName):
    configParameters = {} # Initialize an empty dictionary to store the configuration parameters
    
    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        
        # Split the line
        splitWords = i.split(" ")
        
        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 2
        
        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            digOutSampleRate = int(splitWords[11])
            numAdcSamplesRoundTo2 = 1;
            
            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2;
                
            digOutSampleRate = int(splitWords[11]);
            
        # Get the information about the frame configuration    
        elif "frameCfg" in splitWords[0]:
            
            chirpStartIdx = int(splitWords[1]);
            chirpEndIdx = int(splitWords[2]);
            numLoops = int(splitWords[3]);
            numFrames = int(splitWords[4]);
            framePeriodicity = int(splitWords[5]);

            
    # Combine the read data to obtain the configuration parameters           
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate)/(2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)
    
    return configParameters 
# ------------------------------------------------------------------

# Funtion to read and parse the incoming data
def readAndParseData16xx(Dataport: serial.Serial, configParameters):
    global byteBuffer, byteBufferLength
    
    # Constants
    OBJ_STRUCT_SIZE_BYTES = 12;
    BYTE_VEC_ACC_MAX_SIZE = 2**15;
    MMWDEMO_UART_MSG_POINT_CLOUD_2D = 6;
    MMWDEMO_UART_MSG_TARGET_LIST_2D = 7;
    MMWDEMO_UART_MSG_TARGET_INDEX_2D = 8;
    maxBufferSize = 2**15;
    tlvHeaderLengthInBytes = 8;
    pointLengthInBytes = 16;
    targetLengthInBytes = 68;
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    targetDetected = 0 # Checks if a person has been detected
    frameNumber = 0
    targetObj = {}
    pointObj = {}
    
    #readBuffer = Dataport.read(Dataport.in_waiting)
    readBuffer = b''
    while Dataport.inWaiting()>0:
        readBuffer += Dataport.read(Dataport.in_waiting)

    #print(readBuffer)
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    
    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
        byteBufferLength = byteBufferLength + byteCount
        
    # Check that the buffer has some data
    if byteBufferLength > 16:
    
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]
    
        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc + 8]
            if np.all(check == magicWord):
                startIdx.append(loc)
    
        # Check that startIdx is not empty
        if startIdx:
    
            # Remove the data before the first start index
            if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength - startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength-startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength-startIdx[0]:]),dtype = 'uint8')
                byteBufferLength = byteBufferLength - startIdx[0]
    
            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
    
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
    
            # Read the total packet length
            totalPacketLen = np.matmul(byteBuffer[20:20 + 4], word)
    
            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    
    # If magicOK is equal to 1 then process the message
    if magicOK:
        # word array to convert 4 bytes to a 32 bit number
        word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
    
        # Initialize the pointer index
        idX = 0
    
        # Read the header
        # Read the header
        magicNumber = byteBuffer[idX:idX + 8]
        idX += 8
        version = format(np.matmul(byteBuffer[idX:idX + 4], word), 'x')
        idX += 4
        platform = format(np.matmul(byteBuffer[idX:idX + 4], word), 'x')
        idX += 4
        timeStamp = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 4
        totalPacketLen = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 4
        frameNumber = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 4
        subFrameNumber = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 4
        chirpMargin = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 4
        frameMargin = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 4
        uartSentTime = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 4
        trackProcessTime = np.matmul(byteBuffer[idX:idX + 4], word)
        idX += 4
    
        word = [1, 2 ** 8]
    
        numTLVs = np.matmul(byteBuffer[idX:idX + 2], word)
        idX += 2
        checksum = np.matmul(byteBuffer[idX:idX + 2], word)
        idX += 2
    
        # Read the TLV messages
        for tlvIdx in range(numTLVs):
        # word array to convert 4 bytes to a 32 bit number
            word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
            
            # Initialize the tlv type
            tlv_type = 0
    
            try: 
                # Check the header of the TLV message
                tlv_type = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
                if tlv_type == MMWDEMO_UART_MSG_POINT_CLOUD_2D:
                    print("byteBuffer[idX:idX + 4] ==", byteBuffer[idX:idX + 4])
                tlv_length = np.matmul(byteBuffer[idX:idX + 4], word)
                idX += 4
            except:
                pass
    
            # Read the data depending on the TLV message
            if tlv_type == MMWDEMO_UART_MSG_POINT_CLOUD_2D:
                # word array to convert 4 bytes to a 16 bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
    
                # Calculate the number of detected points
                print("tlv_length>>", tlv_length)
                numInputPoints = (tlv_length - tlvHeaderLengthInBytes) // pointLengthInBytes
    
                # Initialize the arrays
                rangeVal = np.zeros(numInputPoints, dtype=np.float32)
                azimuth = np.zeros(numInputPoints, dtype=np.float32)
                dopplerVal = np.zeros(numInputPoints, dtype=np.float32)
                snr = np.zeros(numInputPoints, dtype=np.float32)
    
                for objectNum in range(numInputPoints):
                    # Read the data for each object
                    rangeVal[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    azimuth[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    dopplerVal[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    snr[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
    
                    # Store the data in the detObj dictionary
                pointObj = {"numObj": numInputPoints, "range": rangeVal, "azimuth": azimuth,\
                            "doppler": dopplerVal, "snr": snr}
            
                dataOK = 1
    
            elif tlv_type == MMWDEMO_UART_MSG_TARGET_LIST_2D:
    
                # word array to convert 4 bytes to a 16 bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]
    
                # Calculate the number of target points
                numTargetPoints = (tlv_length - tlvHeaderLengthInBytes) // targetLengthInBytes
    
                # Initialize the arrays
                targetId = np.zeros(numTargetPoints, dtype=np.uint32)
                posX = np.zeros(numTargetPoints, dtype=np.float32)
                posY = np.zeros(numTargetPoints, dtype=np.float32)
                velX = np.zeros(numTargetPoints, dtype=np.float32)
                velY = np.zeros(numTargetPoints, dtype=np.float32)
                accX = np.zeros(numTargetPoints, dtype=np.float32)
                accY = np.zeros(numTargetPoints, dtype=np.float32)
                EC = np.zeros((3, 3, numTargetPoints), dtype=np.float32)  # Error covariance matrix
                G = np.zeros(numTargetPoints, dtype=np.float32)  # Gain
    
                for objectNum in range(numTargetPoints):
                # Read the data for each object
                    targetId[objectNum] = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                    posX[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    posY[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    velX[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    velY[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    accX[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    accY[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[0, 0, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[0, 1, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[0, 2, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[1, 0, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[1, 1, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[1, 2, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[2, 0, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[2, 1, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    EC[2, 2, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    G[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
    
                # Store the data in the detObj dictionary
                targetObj = {"targetId": targetId, "posX": posX, "posY": posY, \
                             "velX": velX, "velY": velY, "accX": accX, "accY": accY, \
                             "EC": EC, "G": G, "numTargets":numTargetPoints}
                
                targetDetected = 1
                print(targetDetected)
    
            elif tlv_type == MMWDEMO_UART_MSG_TARGET_INDEX_2D:
                # Calculate the length of the index message
                numIndices = tlv_length - tlvHeaderLengthInBytes
                indices = byteBuffer[idX:idX + numIndices]
                idX += numIndices
    
    
        # Remove already processed data
        if idX > 0:
            shiftSize = totalPacketLen
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),dtype = 'uint8')
            byteBufferLength = byteBufferLength - shiftSize
    
            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0
                

    return dataOK, targetDetected, frameNumber, targetObj, pointObj

# ------------------------------------------------------------------

# Funtion to update the data and display in the plot
def update():
    
    dataOk = 0
    targetDetected = 0
    global targetObj
    global pointObj
    x = []
    y = []
      
    # Read and parse the received data
    dataOk, targetDetected, frameNumber, targetObj, pointObj = readAndParseData16xx(Dataport, configParameters)
    #print(dataOk, targetDetected, frameNumber, targetObj, pointObj)
    
    if targetDetected:
        print(targetObj)
        print(targetObj["numTargets"])
        x = -targetObj["posX"]
        y = targetObj["posY"]
        s2.setData(x,y)
        QtWidgets.QApplication.processEvents()
        
    if dataOk: 
        x = -pointObj["range"]*np.sin(pointObj["azimuth"])
        y = pointObj["range"]*np.cos(pointObj["azimuth"])
        
        s1.setData(x,y)
        QtWidgets.QApplication.processEvents()

    plot.setRange(xRange=[min(x, default=0), max(x, default=100)], yRange=[min(y, default=0), max(y, default=100)])
    
    return dataOk

import sys
def show_test_graph():
    # Create the application and main window
    app = QtWidgets.QApplication(sys.argv)
    window = QtWidgets.QMainWindow()
    window.setWindowTitle("Simple Test Graph")

    # Create a container widget for the graph
    central_widget = QtWidgets.QWidget()
    window.setCentralWidget(central_widget)

    # Create a vertical layout for the container widget
    layout = QtWidgets.QVBoxLayout(central_widget)

    # Create a widget to hold the graph
    graph_widget = pg.PlotWidget(background="w")
    layout.addWidget(graph_widget)

    # Generate some sample data
    x = [0, 1, 2, 3, 4, 5]
    y = [0, 1, 4, 9, 16, 25]

    # Plot the data
    graph_widget.plot(x, y, pen='b', symbol='o', symbolSize=8, symbolPen='b', symbolBrush='r')

    # Show the main window
    window.show()

    # Start the application event loop
    sys.exit(app.exec_())

#show_test_graph()

# -------------------------    MAIN   -----------------------------------------  

# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)

# START QtAPPfor the plot
app = QtWidgets.QApplication([])

# Set the plot 
pg.setConfigOption('background','w')
window = QtWidgets.QMainWindow()
window.setWindowTitle("2D scatter plot")
# Create a container widget for the graph
central_widget = QtWidgets.QWidget()
window.setCentralWidget(central_widget)
#win = pg.GraphicsLayoutWidget(title="2D scatter plot")
plot = pg.PlotWidget(background="w")
# Create a vertical layout for the container widget
layout = QtWidgets.QVBoxLayout(central_widget)
layout.addWidget(plot)

#p = win.addPlot()
p = plot
p.setXRange(-0.5,0.5)
p.setYRange(0,6)
p.setLabel('left',text = 'Y position (m)')
p.setLabel('bottom', text= 'X position (m)')
s1 = p.plot([],[],pen=None,symbol='o')
s2 = p.plot([],[],pen=(0,0,255),symbol='star')

window.show()


# Main loop 
targetObj = {}  
pointObj = {}
frameData = {}    
currentIndex = 0
def loop_step():
    try:
        # Update the data and check if the data is okay
        dataOk = update()
        
        if dataOk:
            # Store the current frame into frameData
            frameData[currentIndex] = targetObj
            currentIndex += 1
        
        #time.sleep(0.033) # Sampling frequency of 30 Hz

    # Stop the program and close everything if Ctrl + c is pressed
    except KeyboardInterrupt:
        #print("currentIndex =", currentIndex)
        #print("total frames =", len(frameData))
        CLIport.write(('sensorStop\n').encode())
        CLIport.close()
        Dataport.close()
        window.close()
        return

from PyQt5.QtCore import QTimer
# Set up the QTimer to trigger the update_plot function every 100 milliseconds
update_timer = QTimer()
update_timer.timeout.connect(loop_step)
update_timer.start(33)  # Update the plot every 100 milliseconds (adjust this interval as needed)

# Start the application event loop
sys.exit(app.exec_())