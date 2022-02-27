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

import time
from BuildingSensor import BuildingSensor
from AreaSensor import AreaSensor

class SnowPlow():
    def __init__(self, ID):
        super(SnowPlow, self).__init__()
        # Assign clientID to global variable
        self.clientID = ID
        # Get handles for four joints
        _, self.leftFrontJointHandle = sim.simxGetObjectHandle(self.clientID, 'LeftFrontJoint',
                                                               sim.simx_opmode_blocking)
        _, self.leftBackJointHandle = sim.simxGetObjectHandle(self.clientID, 'LeftBackJoint', sim.simx_opmode_blocking)
        _, self.rightFrontJointHandle = sim.simxGetObjectHandle(self.clientID, 'RightFrontJoint',
                                                                sim.simx_opmode_blocking)
        _, self.rightBackJointHandle = sim.simxGetObjectHandle(self.clientID, 'RightBackJoint',
                                                               sim.simx_opmode_blocking)
        # Initial velocity to four wheels
        self.initialVelocity = 2
        # Position variable that will track the vehicle position later
        self.position = None
        # Keep track the last turn direction: 0-left, 1-right
        self.lastTurn = None
        # Keep track the Scan Mode loop: 0: left-forward-left loop, 1: right-forward-right loop
        self.ScanModeLoop = 0
        # Instantiate each sensor
        #self.buildingSensor = BuildingSensor(self.clientID)
        self.areaSensor = AreaSensor(self.clientID)
        # Start each sensor thread
        ##self.buildingSensor.start()
        self.areaSensor.start()
        self.run()

    # Start executing the performance
    def run(self):
        # Start different modes
        #self.ScanMode()
        self.setInitVelocity(self.initialVelocity)
        time.sleep(5)
        self.directionTurning("right", 200)
        while 1:
        #     _, self.position = sim.simxGetObjectPosition(self.clientID, self.buildingSensorHandle, -1,
        #                                                  sim.simx_opmode_streaming)
        #     print(self.position)
            self.setInitVelocity(self.initialVelocity)
        # Bye and termination
        sim.simxAddStatusbarMessage(self.clientID, 'Bye CoppeliaSim!!', sim.simx_opmode_oneshot)
        sim.simxGetPingTime(self.clientID)
        sim.simxFinish(self.clientID)

    # Assign a specific velocity to the vehicle to move forward
    # @param: velocity: int: specific velocity assigned to the vehicle
    def setInitVelocity(self, velocity):
        sim.simxSetJointTargetVelocity(self.clientID, self.leftBackJointHandle, velocity, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.leftFrontJointHandle, velocity, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.rightBackJointHandle, velocity, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.rightFrontJointHandle, velocity, sim.simx_opmode_blocking)

    # Let the car keep moving forward if building sensor and area sensor doesn't detect anything
    # Only get out of this function when building sensor or area sensor detects something
    # @param: sleeptime: init: the time limit to move forward, if no time limit, sleeptime = 0
    # TODO: need to update the perform when each sensor sends data
    def moveForward(self, sleeptime):
        # Set initial velocity to let the car keep moving forward
        self.setInitVelocity(self.initialVelocity)
        # If no forwarding time set
        # Only need to check
        if sleeptime == 0:
            # Infinite loop
            # Stuck in the loop when building sensor doesn't detect anything
            # Gets out when building sensor detects something
            # TODO: need to add area sensor checker later
            while not self.buildingSensor.isObstacleDetected:
                pass
            # TODO: add other performance at this line
        # If there's a time limit set for moving forward
        else:
            # Ensure no obstacle is detected by building sensor
            # TODO: need to add area sensor checker later
            if not self.buildingSensor.isObstacleDetected:
                time.sleep(sleeptime)

    # Function turning the car in different directions for a specific angle
    # @param: direction: left or right in string: the direction the car needs to turn
    # @param: angle: int: the angle the car needs to turn along the direction
    def directionTurning(self, direction, angle):
        # 7 seconds needed to turn 90 degrees
        ninetyDegreeTime = 8
        # Calculate the total time required to turn a desired angle
        turningTime = (angle / 90) * ninetyDegreeTime
        # If the car needs to turn right
        if direction == "right":
            # Update the lastTurn to 1: right
            self.lastTurn = 1
            # Assign left wheels positive velocity and right wheels negative velocity
            sim.simxSetJointTargetVelocity(self.clientID, self.leftBackJointHandle, 2, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.leftFrontJointHandle, 2, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.rightBackJointHandle, -2, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.rightFrontJointHandle, -2, sim.simx_opmode_blocking)
        # If the car needs to turn left
        else:
            # Update the lastTurn to 0: left
            self.lastTurn = 0
            # Assign right wheels positive velocity and left wheels negative velocity
            sim.simxSetJointTargetVelocity(self.clientID, self.leftBackJointHandle, -2, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.leftFrontJointHandle, -2, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.rightBackJointHandle, 2, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.rightFrontJointHandle, 2, sim.simx_opmode_blocking)
        # Wait a period of time for the car to turn
        time.sleep(turningTime)
        # After turning a specific angle change the four wheels velocity back to 2m/s forward
        self.setInitVelocity(self.initialVelocity)

    # Define the Scan Mode
    # Allows the vehicle to scan the area in a "S"-shape path
    # TODO: need to add area sensor and snow sensor to this function later
    def ScanMode(self):
        # Let the car move forward for 2 seconds when it first starts
        self.moveForward(6)
        self.directionTurning("right", 90)
        self.moveForward(20)#TODO
        # Implement the state machine
        # TODO: need to add if statement to check each sensor's data right after each self.moveForward(2)
        while 1:
            # If need to perform a right-forward-right loop first then left-forward-left loop
            if self.ScanModeLoop:# TODO: Update this condition later (should connect to snow sensor)
                # Start right loop first
                self.ScanModeTurnLoop(1)
                # Check sub mode again, ensure it's not changed
                if not self.ScanModeLoop:
                    # If the sub mode changes, do the previous turn one more time
                    self.ScanModeTurnLoop(1)
                    # Change the sub mode back to keep the loop within the if statement
                    self.ScanModeLoop = (self.ScanModeLoop + 1) % 2
                # Perform a left turn
                self.ScanModeTurnLoop(0)
            # If need to perform a left-forward-left loop first then right-forward-right loop
            else:
                # Start left-forward-left loop first
                self.ScanModeTurnLoop(0)
                # Check sub mode again, ensure it's not changed
                if self.ScanModeLoop:
                    # If the sub mode changes, do the previous  one more time
                    self.ScanModeTurnLoop(0)
                    # Change the sub mode back to keep the loop within the if statement
                    self.ScanModeLoop = (self.ScanModeLoop + 1) % 2
                # Perform a right turn
                self.ScanModeTurnLoop(1)

    # Sub-loop for Scan Mode
    # @param direction: 1: right-forward-right loop, 0: left-forward-left loop
    # Called by function ScanMode()
    def ScanModeTurnLoop(self, direction):
        # Assign string depending on the direction
        if direction:
            string = "right"
        else:
            string = "left"

        # Start the loop
        self.directionTurning(string, 90)
        self.moveForward(5)
        # Checks if any Obstacle is detected
        # TODO: need to add conditions for area sensors later
        if self.buildingSensor.isObstacleDetected:
            # If yes, update the ScanMode_SubMode 1 to 0, or 0 to 1
            # Enter another sub-mode
            self.ScanModeLoop = (self.ScanModeLoop + 1) % 2
            # Exit this function
            return None
        self.directionTurning(string, 90)
        self.moveForward(40)#TODO


if __name__ == "__main__":
    # Try to connect the host at port 19999
    sim.simxFinish(-1)
    # connect to the server socket
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    # if connected to the server
    if clientID != -1:
        # Display connected to server info
        print('Connected to remote API server')
        SnowPlow(clientID)
    # if not connected to the server
    else:
        print('Failed connecting to remote API server')
    print('Program ended')
