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


class BuildingSensor(threading.Thread):
    # Single indicates car moveAlgorithm when building sensor detects anything
    # Will be checked by MoveAlgorithm class
    isObstacleDetected = False

    def __init__(self, ID):
        threading.Thread.__init__(self)
        # Define variables required to read the building sensor
        self.clientID = ID
        # Get the building sensor handle
        _, self.buildingSensorHandle = sim.simxGetObjectHandle(self.clientID, 'BuildingSensor',
                                                               sim.simx_opmode_blocking)
        # Counts how many time the sensor function has been called
        self.counter = 0
        # Checks if building sensor detects anything True/False
        self.detectState = None
        # Holds the distance returned by the building sensor
        self.distance = None

    def run(self):
        while 1:
            # If the function is called for the first time
            # Use simx_opmode_streaming mode
            if self.counter == 0:
                returnCode, self.detectState, self.distance, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                    self.clientID, self.buildingSensorHandle, sim.simx_opmode_streaming)
            # If the function is called after the first time
            # Use simx_opmode_buffer mode
            else:
                returnCode, self.detectState, self.distance, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                    self.clientID, self.buildingSensorHandle, sim.simx_opmode_buffer)
            # Update the counter
            self.counter += 1
            # If an obstacle is detected by the building sensor
            if self.detectState:
                # If the distance is less than 0.5 meter
                if self.distance[2] < 0.5:
                    # Update the variable
                    self.isObstacleDetected = True
