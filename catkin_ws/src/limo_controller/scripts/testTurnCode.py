import numpy as np


TURN_ANGLE_RANGE = 30 # degrees
TURN_CLOSEST_PERCENT = 10 # percent


def main(turnDistances):
# ----- Turning implementation 
    # Declare variables for the implementation
    steeringMatrix = np.empty(len(turnDistances)) # array that holds steering angle with same indexes as the turn distances

    # Calculate the degree each datapoint is relative
    totalDegreesCovered = TURN_ANGLE_RANGE * 2
    distBetweenMeasurements = totalDegreesCovered / (len(turnDistances) - 1)
    
    # Calculate the steering angle towards each datapoint and insert into an array

    turnAngle = TURN_ANGLE_RANGE * -1
    for index, dataPoint in enumerate(turnDistances):
        """
        Caculation here.
        """
        
        # REPLACE WITH CALCULATED EQUATION RESULT - Currently saves degrees of each value
        steeringMatrix[index] = turnAngle 
        turnAngle = turnAngle + distBetweenMeasurements

    # Calculate the what datapoints are the closest (ex. 90% closest datapoints - FINE TUNE PERCENTAGE)
    numCloseValues = int((TURN_CLOSEST_PERCENT/100) * len(turnDistances)) # number of values in the top * percent
    sortedDistances = sorted(turnDistances)
    closestDistances = sortedDistances[:numCloseValues]

    indexArr = [turnDistances.index(value) for value in closestDistances] # the index values of the closest values

    # Average the steering angle towards these datapoints to get the needed steering angle
    closestAngles = []
    for index in indexArr:
        closestAngles.append(steeringMatrix[index])

    averageSteeringAngle = sum(closestAngles)/len(closestAngles)
    
    return averageSteeringAngle # Return the steering angle based on LiDAR data

if __name__ == '__main__':
    turnDistances = [10.1, 10.2, 12.5, 20.3, 20.0, 21.1, 21.1, 10.0, 10.0, 10.3, 11.2, 13.1, 16.4, 20.1, 21.2, 6.3, 3.1, 4.1, 5.0, 4.1, 6.2, 7.3]
    main(turnDistances)