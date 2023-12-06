
LEFT_SENSOR_VAL = .85956 # all positive values are left
RIGHT_SENSOR_VAL = -0.5576 # all negative values are right

TURN_ANGLE_RANGE = 35 # degrees
TURN_CLOSEST_PERCENT = 15 # percent

'''
Check if scan is done clockwise or counterclockwise.
Check maximum turn radius of limo
Check if turn sensor values need to be fine tuned for each limo
'''

def main(turnDistances):
# ----- Turning implementation 
    # Declare variables for the implementation
    steeringMatrix = [0] * len(turnDistances) # array that holds steering angle with same indexes as the turn distances
    numHalfTurnAngle = int(int(len(turnDistances))/2)
    
    # Calculate the distances for the right and left sides of the data set
    distBetweenMeasurementsLeft = LEFT_SENSOR_VAL / numHalfTurnAngle
    distBetweenMeasurementsRight = RIGHT_SENSOR_VAL / numHalfTurnAngle
    
    # Calculate the steering angle towards each datapoint and insert into an array
    turnAngle = LEFT_SENSOR_VAL
    for index, dataPoint in enumerate(turnDistances):
        steeringMatrix[index] = turnAngle

        if index < numHalfTurnAngle: # handle the left side
            turnAngle = turnAngle - distBetweenMeasurementsLeft
        elif index > numHalfTurnAngle: # handle the right side
            turnAngle = turnAngle + distBetweenMeasurementsRight
        elif index == numHalfTurnAngle: # handle the center
            turnAngle = 0

    # Calculate the what datapoints are the closest (ex. 90% closest datapoints - FINE TUNE PERCENTAGE)
    numCloseValues = int((TURN_CLOSEST_PERCENT/100) * len(turnDistances)) # number of values in the top * percent
    sortedDistances = sorted(turnDistances)
    closestDistances = sortedDistances[:numCloseValues]

    indexArr = [turnDistances.index(value) for value in closestDistances] # the index values of the closest values

    # Average the steering angle towards these datapoints to get the needed steering angle
    closestAngles = []
    for index in indexArr:
        closestAngles.append(steeringMatrix[index])

    global steeringAngle
    steeringAngle = sum(closestAngles)/len(closestAngles)
    
    return steeringAngle # Return the steering angle based on LiDAR data

if __name__ == '__main__':
    turnDistances = [10.1, 10.2, 12.5, 20.3, 20.0, 21.1, 21.1, 10.0, 10.0, 10.1, 10.2, 10.3, 11.2, 13.1, 16.4, 20.1, 21.2, 6.3, 3.1, 4.1, 5.0, 4.1, 6.2, 7.3]
    
    result = main(turnDistances)
    print(result)
