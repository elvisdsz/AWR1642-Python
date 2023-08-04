import serial
import time

serial_port = 'COM12'
baud_rate = 921600

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

# Establish a connection to the serial port
# ser = serial.Serial(serial_port, baud_rate, timeout=1)



import numpy as np
def process_bytes(byteBuffer):

    byteBuffer = [byte for byte in byteBuffer]
    byteBuffer = np.array(byteBuffer, dtype = 'uint8')

    MMWDEMO_UART_MSG_POINT_CLOUD_2D = 6
    MMWDEMO_UART_MSG_TARGET_LIST_2D = 7
    MMWDEMO_UART_MSG_TARGET_INDEX_2D = 8
    maxBufferSize = 2**15
    tlvHeaderLengthInBytes = 8
    pointLengthInBytes = 16
    targetLengthInBytes = 68
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]
    
    # Initialize variables
    magicOK = 0 # Checks if magic number has been read
    dataOK = 0 # Checks if the data has been read correctly
    targetDetected = 0 # Checks if a person has been detected
    frameNumber = 0
    targetObj = {}
    pointObj = {}

    byteBufferLength = len(byteBuffer)

    if byteBufferLength > 16:
        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]
        # if len(possibleLocs) > 0:
        #    print("possibleLocs>>", possibleLocs)

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc + 8]
            if len(check)==8 and np.all(check == magicWord):
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
        print("Magic OK!")
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
                print("tlv_type == ", tlv_type)
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

    #return
    if dataOK:
        print("RESULT>>", dataOK, targetDetected, frameNumber, targetObj, pointObj)





import os
# cfg_filename = 'profile_2023_08_02T11_35_33_607.cfg'
cfg_filename = 'all_plots_1fps.cfg'
script_dir = os.path.dirname(__file__)
configFilePath = os.path.join(script_dir, cfg_filename)
cliport, dataport = serialConfig(configFilePath)


filename = "D:\\HE\\soton\\acad\\MScProject\\radar\\ti\\tests\\py_rec\\awr1642_data_out.txt"
rec_duration = 10 # seconds

start_time = time.time()
max_time = start_time + rec_duration
sensor_stopped = False
try:
    with open(filename, "wb") as f:
        print("Recording to file >> ", filename)
        while True:
            if (not sensor_stopped) and time.time() > max_time:
                print("Time's up! -- sensorStop")
                cliport.write(("sensorStop"+'\n').encode())
                sensor_stopped = True

            # Read data from the serial port
            # data = dataport.readline().decode('utf-8').strip()
            # data = int(dataport.readline().decode('utf-8'))
            data = Dataport.read(Dataport.in_waiting)

            # If data received, print it
            if data:
                # data = int.from_bytes(data, "big")
                # print("Received data from serial port: ", data)
                # Give the device time to send data again
                #time.sleep(0.5)
                print(data)
                f.write(data)
                # *** V uncomment this to process
                # process_bytes(data)

            if time.time() > max_time+2 and not data:
                print("Finishing recording . . .")
                break

finally:
    print("Finished recording for ", (time.time()-start_time), " seconds -- saved to "+filename)
    if not sensor_stopped:
        print("sensorStop")
        cliport.write(("sensorStop"+'\n').encode())
    print("Closing the serial port.")
    dataport.close()
    cliport.close()   
            

'''
# To close the serial port gracefully, use Ctrl+C to break the loop
except KeyboardInterrupt:
    print("Closing the serial port.")
    dataport.close()
    cliport.close()
'''


