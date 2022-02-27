try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

import threading


class AreaSensor(threading.Thread):
    # Indicates the vehicle moveAlgorithm when any of the three area sensors detects anything
    # Will be checked by MoveAlgorithm class
    # [Left, Middle, Right]
    areaSensorList = [False, False, False]
    # Set to 0 only when all area sensors are OFF, 1-left only, 2-middle only, 3-right only, 12-left,middle,
    # 23-middle, right, 123-left,middle,right
    isAreaDetected = 0

    def __init__(self, ID):
        threading.Thread.__init__(self)
        # Define variables required to read the building sensor
        self.ID = ID
        # Get the handles for three area sensors
        _, self.leftAreaSensorHandle = sim.simxGetObjectHandle(self.ID, 'AreaSensorLeft',
                                                               sim.simx_opmode_blocking)
        _, self.middleAreaSensorHandle = sim.simxGetObjectHandle(self.ID, 'AreaSensorMiddle',
                                                                 sim.simx_opmode_blocking)
        _, self.rightAreaSensorHandle = sim.simxGetObjectHandle(self.ID, 'AreaSensorRight',
                                                                sim.simx_opmode_blocking)
        # Hold the data returned by the left, middle, right area sensors
        self.dataLeft = None
        self.dataMiddle = None
        self.dataRight = None

    def run(self):
        while 1:
            # Read each vision sensor and get data
            _, detectStateLeft, self.dataLeft, = sim.simxReadVisionSensor(
                self.ID, self.leftAreaSensorHandle, sim.simx_opmode_blocking)
            _, detectStateMiddle, self.dataMiddle, = sim.simxReadVisionSensor(
                self.ID, self.middleAreaSensorHandle, sim.simx_opmode_blocking)
            _, detectStateRight, self.dataRight, = sim.simxReadVisionSensor(
                self.ID, self.rightAreaSensorHandle, sim.simx_opmode_blocking)
            # if left area sensor detects black color
            if self.dataLeft[0][11] < 0.3:
                self.areaSensorList[0] = True
            else:
                self.areaSensorList[0] = False
            # if middle area sensor detects black color
            if self.dataMiddle[0][11] < 0.3:
                self.areaSensorList[1] = True
            else:
                self.areaSensorList[1] = False
            # if right area sensor detects black color
            if self.dataRight[0][11] < 0.3:
                self.areaSensorList[2] = True
            else:
                self.areaSensorList[2] = False

            # Update variable isAreaDetected
            self.signalDecoder()

            print(self.areaSensorList)
            print("\n")

    # Update the variable isAreaDetected
    def signalDecoder(self):
        # If none of the area sensor detects area
        if not self.areaSensorList[0] and not self.areaSensorList[1] and not self.areaSensorList[2]:
            self.isAreaDetected = 0
        # If only left area sensor detects area
        elif self.areaSensorList[0] and not self.areaSensorList[1] and not self.areaSensorList[2]:
            self.isAreaDetected = 1
        # If only middle area sensor detects area
        elif not self.areaSensorList[0] and self.areaSensorList[1] and not self.areaSensorList[2]:
            self.isAreaDetected = 2
        # If only right area sensor detects area
        elif not self.areaSensorList[0] and not self.areaSensorList[1] and self.areaSensorList[2]:
            self.isAreaDetected = 3
        # If left and middle area sensors detect area
        elif self.areaSensorList[0] and self.areaSensorList[1] and not self.areaSensorList[2]:
            self.isAreaDetected = 12
        # If middle and right area sensors detect area
        elif not self.areaSensorList[0] and self.areaSensorList[1] and self.areaSensorList[2]:
            self.isAreaDetected = 23
        # If all three area sensors detect area
        elif self.areaSensorList[0] and self.areaSensorList[1] and self.areaSensorList[2]:
            self.isAreaDetected = 123
