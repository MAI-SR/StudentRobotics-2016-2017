###################################################################################################################
#READ ME: For this to work you need to:
#        	-have your forwardfacing camera, that is at the turning center of your robot (the robot MUST turn around a center, else the estimateArenaAngle methode is useless)
#			-implement estimateArenaAngle into every turn methode
#			-implement estimateArenaPosition into every drive methode
#			-implement debugging into every methode changing any variables in it
#			-use the takeAndProcessPicture methode whenever taking a picture (it even tells you what markers it found ;) )
#		  I recommend you:
#			-always use the estimatedArena values, seeing as if your camera dosen't detect an arenaToken the arena values will be outdated, while your estimatedArena values will at least be somewhat close
###################################################################################################################


###################################################################################################################
#Imports:
###################################################################################################################

from sr.robot import *
from math import *
import ll
import time

###################################################################################################################
#Low Level API:
###################################################################################################################

def drive(distance):#use meters(float); positive digits to move forward, negative digits to move backwards
    return ll.lowLevelDrive(distance)
    
def rotate(angle):#use degrees(float)
    return ll.lowLevelRotate(angle)
    
def turn(radius):#use meters(float)
    return ll.lowLevelTurn(radius)
    
def arm(state):#use "open" or "close"    
    if state=="open":
        return ll.lowLevelServoState("open")
    elif state=="close":
        return ll.lowLevelServoState("close")
        
def readUS():
    return ll.CustomisedRuggeduino().readUS()
    
###################################################################################################################
#Constants:
###################################################################################################################

arenaMarkerConstantX = [-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 3.0, 2.0, 1.0, 0.0, -1.0, -2.0, -3.0, -4.0, -4.0, -4.0, -4.0, -4.0, -4.0, -4.0]#this is a constant used in calculateArenaPosition
arenaMarkerConstantY = [4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 3.0, 2.0, 1.0, 0.0, -1.0, -2.0, -3.0, -4.0, -4.0, -4.0, -4.0, -4.0, -4.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0]#this is a constant used in calculateArenaPosition

tokenWidth = 0.1225#this is the width of a token in meters divided by 2
tokenTiltTolerance = 10#this is the tolerance in degrees for a token to still be considerd upright (for example 90 +/-tokenTiltTolerance)

robotCameraResolutionX = 1280#this is the robot cameras X resolution
robotCameraResolutionY = 1024#this is the robot cameras Y resolution

zoneReturnX = [-2.75, 2.75, 2.75, -2.75]#this is a constant used in driveToZone
zoneReturnY = [2.75, 2.75, -2.75, -2.75]#this is a constant used in driveToZone
zoneReturnAngle = [135.0, 45.0, 315.0, 225.0]#this is a constant used in driveToZone

zoneTokenSpotX = [-3.0, 2.5, 3.0, -2.5]#this is a constant used in adjustPosition
zoneTokenSpotY = [2.5, 3.0, -2.5, -3.0]#this is a constant used in adjustPosition
zoneTokenSpotAngle = [336.0, 246.0, 156.0, 66.0]#this is a constant used in adjustPosition

zoneSecureTokenX = [[-3.0, -2.5, -3.5, -2.5, -2.0, -3.5], [3.5, 3.5, 2.5, 3.0, 3.5, 2.0], [3.0, 2.5, 3.5, 2.5, 2.0, 3.5], [-3.5, -3.5, -2.5, -3.0, -3.5, -2.0]]#this is a constant used in storeToken
zoneSecureTokenY = [[3.5, 3.5, 2.5, 3.0, 3.5, 2.0], [3.0, 2.5, 3.5, 2.5, 2.0, 3.5], [-3.5, -3.5, -2.5, -3.0, -2.5, -2.0], [-3.0, -2.5, -3.5, -2.5, -2.0, -3.5]]#this is a constant used in storeToken
zoneSecureTokenAngle = [[0.0, 0.0, 270.0, 0.0, 0.0, 270.0], [270.0, 270.0, 180.0, 270.0, 270.0, 180.0], [180.0, 180.0, 90.0, 180.0, 180.0, 90.0], [90.0, 90.0, 0,0, 90.0, 90.0, 0,0]]#this is a constant used in storeToken



###################################################################################################################
#Variables:
###################################################################################################################

arenaPositionX = 0.0
arenaPositionY = 0.0
arenaAngle = 0.0

estimatedArenaPositionX = 0.0
estimatedArenaPositionY = 0.0
estimatedArenaAngle = 0.0

tokens = []
otherRobots = []

securedTokens = []#this will only be the tokens code so it is an array of intedgers
holdingToken = -1#this will only be the tokens code so it is just an intedgers (-1 = invalid)
targetingToken = None#this will be an objekt of the class Token
hasA = False
hasB = False
hasC = False
ourScore = 0

robotZone = -1

state = "beginning"#this is the state used for the statemaschine in main()

###################################################################################################################
#Debugging Constants and Options:
###################################################################################################################

maximumSeenTokens = 6#this is the maximum number of tokens that the robot can see in one picture
maximumSeenOtherRobots = 6#this is the maximum number of robotMarkers that the robot can see in one picture



###################################################################################################################
#Classes:
###################################################################################################################

class Token:
    tokenMarker = None#this is an array of the known markers for a certain token
	tokenMarkerArenaAngle = 0.0#this the lowest angle of the middle of a marker from this token (by adding 0, 90 ,180 or 270 you can get all the angles for the direction the markersa are facing)
	tokenArenaPositionX = 0.0#this is the tokens middle
	tokenArenaPositionY = 0.0#this is the tokens middle
	tokenMarkerDirection = 'unassigned'#this is the direction the marker is facing ('up    'or'middle')
	tokenType = 'unassigned'#this is the type of the token (a,b,c)
	def __init__(self, tokenMarker, tokenMarkerArenaPositionX, tokenMarkerArenaPositionY, tokenMarkerArenaAngle, tokenMarkerDirection):#tokenMarkerArenaAngle is facing inward not outward
		global tokenWidth
		self.tokenMarker = tokenMarker
		self.tokenMarkerArenaAngle = tokenMarkerArenaAngle%90.0
		self.tokenMarkerDirection = tokenMarkerDirection
		if tokenMarker.info.token_net == "NET_A":
			self.tokenType = 'a'
		elif tokenMarker.info.token_net == "NET_B":
			self.tokenType = 'b'
		elif tokenMarker.info.token_net == "NET_C":
			self.tokenType = 'c'
		if tokenMarkerDirection == 'middle':
			hypotenuse = tokenWidth
			angle = math.radians(tokenMarkerArenaAngle)
			self.tokenArenaPositionX = math.cos(angle)*hypotenuse+tokenMarkerArenaPositionX
			self.tokenArenaPositionY = math.sin(angle)*hypotenuse+tokenMarkerArenaPositionY
		else:#tokenMarkerDirection == 'up'
			self.tokenArenaPositionX = tokenMarkerArenaPositionX
			self.tokenArenaPositionY = tokenMarkerArenaPositionY


class OtherRobot:
	robotCorner = -1#this is the corner belonging to the robot
	robotMarkerArenaAngle = 0.0#this is the angle the marker of this robot is facing
	robotArenaPositionX = 0.0#this is the robots markers position
	robotArenaPositionY = 0.0#this is the robots markers position
	def __intin__(self, robotArenaPositionX, robotArenaPositionY, robotMarkerArenaAngle, robotMarkerCorner):
		self.robotCorner = robotMarkerCorner
		self.robotMarkerArenaAngle = robotMarkerArenaAngle
		self.robotArenaPositionX = robotArenaPositionX
		self.robotArenaPositionY = robotArenaPositionY


###################################################################################################################
#Management Methodes:
###################################################################################################################

def takePicture():#DO NOT USE!!! takes a picture and returns an array of markers
	return robot.see((robotCameraResolutionX, robotCameraResolutionY))

def sortMarkersByType(theMarkers):#DO NOT USE!!! sorts an array (markers) and returns 3 arrays (arena, robot, token)
	arenaMarkers = []
	tokenMarkers = []
	robotMarkers = []
	for oneMarker in theMarkers:
		if oneMarker.info.marker_type == "MARKER_ARENA":
			arenaMarkers.append(oneMarker)
		elif oneMarker.info.marker_type == "MARKER_TOKEN_A" or "MARKER_TOKEN_B" or "MARKER_TOKEN_C":
			tokenMarkers.append(oneMarker)
		elif oneMarker.info.marker_type == "MARKER_ROBOT":
			robotMarkers.append(oneMarker)
		else:
			print 'Something went wrong in sortMarkersByType(theMarkers): oneMarker.info.type !=  MARKER_ARENA/MARKER_TOKEN_TOP/MARKER_TOKEN_SIDE/MARKER_TOKEN_BOTTOM/MARKER_ROBOT'
	return arenaMarkers, tokenMarkers, robotMarkers

def sortMarkersByDistance(theMarkers):#DO NOT USE!!! sorts and returns an array (markers) by their distance (this is using insertion sorting)
    sortedMarkers = []
    if len(theMarkers) == 0:
        return sortedMarkers
    sortedMarkers.append(theMarkers[0])
    theMarkers.remove(theMarkers[0])
    for oneMarker in theMarkers:
        inserted = False
        for currentLoopMarker in range(0, len(sortedMarkers)):
            if sortedMarkers[currentLoopMarker].dist > oneMarker.dist and inserted == False:
                sortedMarkers.insert(currentLoopMarker, oneMarker)
                inserted = True
        if inserted == False:
            sortedMarkers.append(oneMarker)
    return sortedMarkers

def calculateArenaAngle(arenaMarker):#DO NOT USE!!! calculates and sets the golbal variable (arenaAngle) based on a marker (arenaMarker)
    robotToMarkerAngle = arenaMarker.orientation.rot_y-arenaMarker.center.rot_y
    markerNumber = arenaMarker.info.code
    absoluteAngle = 0
    if markerNumber < 7 and markerNumber >= 0:
        absoluteAngle = 90.0+robotToMarkerAngle
    elif markerNumber < 14 and markerNumber >= 7:
        absoluteAngle = 0.0+robotToMarkerAngle
    elif markerNumber < 21 and markerNumber >= 14:
        absoluteAngle = 270.0+robotToMarkerAngle
    elif markerNumber <= 27 and markerNumber >= 21:
        absoluteAngle = 180.0+robotToMarkerAngle
    else:
        print 'Something went wrong in calculateArenaAngle(arenaMarker): markerNumber != 0-27'
    absoluteAngle = (absoluteAngle+360.0)%360.0
    arenaAngle = absoluteAngle
    estimatedArenaAngle = arenaAngle

def calculateArenaPosition(arenaMarker):#DO NOT USE!!! calculates and sets the golbal variables (arenaPositionX, arenaPositionY) based on a marker (arenaMarker)
	hypotenuse = arenaMarker.dist
	angle = math.radians(arenaMarker.orientation.rot_y)
	markerNumber = arenaMarker.info.code
	relativX = 0
	relativY = 0
	if markerNumber < 7 and markerNumber >= 0:
		relativX += math.sin(angle)*hypotenuse
		relativY += (-1.0)*abs(math.cos(angle)*hypotenuse)
	elif markerNumber < 14 and markerNumber >= 7:
		relativX += (-1.0)*abs(math.cos(angle)*hypotenuse)
		relativY += math.sin(angle)*hypotenuse*(-1.0)
	elif markerNumber < 21 and markerNumber >= 14:
		relativX += math.sin(angle)*hypotenuse*(-1.0)
		relativY += abs(math.cos(angle)*hypotenuse)
	elif markerNumber <= 27 and markerNumber >= 21:
		relativX += abs(math.cos(angle)*hypotenuse)
		relativY += math.sin(angle)*hypotenuse
	else:
		print 'Something went wrong in calculateArenaPosition(arenaMarker): markerNumber != 0-27'
	arenaPositionX = relativX+arenaMarkerConstantX[markerNumber]
	arenaPositionY = relativY+arenaMarkerConstantY[markerNumber]
	estimatedArenaPositionX = arenaPositionX
	estimatedArenaPositionY = arenaPositionY

def calculateTokenMarkerPositions(tokenMarkers):#DO NOT USE!!! calculates and returns 4 arrays (tokenMarkerArenaPositionsX, tokenMarkerArenaPositionsY, tokenMarkerArenaAngles(this is facing inward not outward), tokenMarkerDirections('up'or'middle')) based on an array (tokenMarkers)
	tokenMarkerArenaPositionsX = []
	tokenMarkerArenaPositionsY = []
	tokenMarkerArenaAngles = []#this is facing inward not outward
	tokenMarkerDirections = []
	for oneTokenMarker in tokenMarkers:
		hypotenuse = oneTokenMarker.dist
		angle = math.radians(((estimatedArenaAngle-oneTokenMarker.center.rot_y)+360.0)%360.0)
		relativX = math.cos(angle)*hypotenuse
		relativY = math.sin(angle)*hypotenuse
		tokenMarkerArenaPositionsX.append(relativX+estimatedArenaPositionX)
		tokenMarkerArenaPositionsY.append(relativY+estimatedArenaPositionY)
		tokenMarkerArenaAngles.append((((math.degrees(angle))-oneTokenMarker.orientation.rot_y)+360.0)%360.0)
		if oneTokenMarker.orientation.rot_x < 90.0+tokenTiltTolerance and oneTokenMarker.orientation.rot_x > 90.0-tokenTiltTolerance:
			tokenMarkerDirections.append('up    ')
		else:
			tokenMarkerDirections.append('middle')
	return tokenMarkerArenaPositionsX, tokenMarkerArenaPositionsY, tokenMarkerArenaAngles, tokenMarkerDirections#tokenMarkerArenaAngles is facing inward not outward

def makeTokens(tokenMarkers, tokenMarkerArenaPositionsX, tokenMarkerArenaPositionsY, tokenMarkerArenaAngles, tokenMarkerDirections):#DO NOT USE!!! this takes all values needed to make tokens, creates all tokens and deleats dobbletokens and then overrides tokens the global variable
    newTokens = []
    oneTokenMarkerNumber = 0
    for oneTokenMarker in tokenMarkers:
        newTokens.append(Token(oneTokenMarker, tokenMarkerArenaPositionsX[oneTokenMarkerNumber], tokenMarkerArenaPositionsY[oneTokenMarkerNumber], tokenMarkerArenaAngles[oneTokenMarkerNumber], tokenMarkerDirections[oneTokenMarkerNumber]))
        oneTokenMarkerNumber += 1
    for oneTokenMarker in newTokens:
        for compairToken in newTokens:
            if oneTokenMarker != compairToken:#so a token cant deleat itself
                if oneTokenMarker.info.code == compairToken.info.code:
                    newTokens.remove(compairToken)
        if len(securedTokens) != 0:
            for oneSecuredToken in securedTokens:
                if oneTokenMarker.info.code == oneSecuredToken:
                    newTokens.remove(oneTokenMarker)
    tokens = newTokens

def updateTargetingToken():#DO NOT USE!!! if there is a targetingToken and a new picture is taken and the targetingToken is in that picture the targetingToken gets replaced by the newly calculated one
    if targetingToken != None:
        for oneToken in tokens:
            if targetingToken.tokenMarker.info.code == oneToken.tokenMarker.info.code:
                targetingToken = oneToken
                print "updated targetingToken"
                return

def calculateRobotMarkerPositions(robotMarkers):#DO NOT USE!!! calculates and returns 4 arrays (robotMarkerArenaPositionX, robotMarkerArenaPositionY, robotMarkerArenaAngle(this is facing inward not outward), robotMarkerCorner) based on an array (robotMarkers)
	robotArenaPositionsX = []
	robotArenaPositionsY = []
	robotMarkerArenaAngles = []#this is facing inward not outward
	robotCorners = []
	for oneRobotMarker in robotMarkers:
		hypotenuse = oneRobotMarker.dist
		angle = math.radians(((estimateArenaAngle-oneRobotMarker.center.rot_y)+360.0)%360.0)
		relativX = math.cos(angle)*hypotenuse
		relativY = math.sin(angle)*hypotenuse
		robotArenaPositionsX.append(relativX+estimatedArenaPositionX)
		robotArenaPositionsY.append(relativY+estimatedArenaPositionY)
		robotMarkerArenaAngles.append((((math.degrees(angle))-oneRobotMarker.orientation.rot_y)+360.0)%360.0)
		robotCorners.append(oneRobotMarker.info.code-28)
	return robotArenaPositionsX, robotArenaPositionsY, robotMarkerArenaAngles, robotCorners#robotMarkerArenaAngle is facing inward not outward

def makeRobots(robotArenaPositionsX, robotArenaPositionsY, robotMarkerArenaAngles, robotCorners):#DO NOT USE!!! this takes all values needed to make otherRobots, creates all otherRobots and then overrides otherRobots the global variable
    newOtherRobots = []
    for robotMarkerNumber in range(0, len(robotCorners)):
        newOtherRobots.append(OtherRobot(robotArenaPositionsX[robotMarkerNumber], robotArenaPositionsY[robotMarkerNumber], robotMarkerArenaAngles[robotMarkerNumber], robotCorners[robotMarkerNumber]))
    otherRobots = newOtherRobots

def takeAndProcessPicture():#takes a picture, calculates the important variables and returns 3 booleans. if they are true there was a new markers of that type. if they are wrong then not
    markers = sortMarkersByType(takePicture())
    arenaMarkers = sortMarkersByDistance(markers[0])
    tokenMarkers = sortMarkersByDistance(markers[1])
    robotMarkers = sortMarkersByDistance(markers[2])
    seenArenaMarker = False
    seenTokenMarker = False
    seenRobotMarker = False
    if len(arenaMarkers) != 0:
        calculateArenaAngle(arenaMarkers[0])
        calculateArenaPosition(arenaMarkers[0])
        seenArenaMarker = True
    if len(tokenMarkers) != 0:
        tokenMarkersInformation = calculateTokenMarkerPositions(tokenMarkers)
        makeTokens(tokenMarkers, tokenMarkersInformation[0], tokenMarkersInformation[1], tokenMarkersInformation[2], tokenMarkersInformation[3])
        updateTargetingToken()
        seenTokenMarker = True
    else:
        clearTokens = []
        tokens = clearTokens
    if len(robotMarkers) != 0:
        robotMarkersInformation = calculateRobotMarkerPositions(robotMarkers)
        makeRobots(robotMarkersInformation[0], robotMarkersInformation[1], robotMarkersInformation[2], robotMarkersInformation[3])
        seenRobotMarker = True
    else:
        clearOtherRobots = []
        otherRobots = clearOtherRobots
    debugging()
    return seenArenaMarker, seenTokenMarker, seenRobotMarker


def estimateArenaAngle(angle):#calculates and sets the estimatedArenaAngle based on an angle (turningangle)
	estimatedArenaAngle = ((estimatedArenaAngle+angle)+360.0)%360.0
	debugging()

def estimateArenaPosition(distance, relativAngle):#calculates and sets the estimatedArenaPositionX and estimatedArenaPositionY based on a distance (drivedistance) and an angle relativ to the robots estimatedArenaAngle (drivedirection)
	hypotenuse = distance
	angle = math.radians(((relativAngle+estimateArenaAngle)+360.0)%360.0)
	relativX = math.cos(angle)*hypotenuse
	relativY = math.sin(angle)*hypotenuse
	estimatedArenaPositionX = estimatedArenaPositionX+relativX
	estimatedArenaPositionY = estimatedArenaPositionY+relativY
	debugging()

 
def calculatePointToPoint(XFrom, YFrom, XTo, YTo):#calculates and returns the distance between 2 points and the absolute angle relativ to the arena
	XDist = XTo-XFrom
	YDist = YTo-YFrom
	hypotenuse = math.sqrt(( XDist*XDist)+(YDist*YDist))
	if XDist == 0 and YDist > 0:
		return hypotenuse, 90.0
	elif XDist == 0 and YDist < 0:
		return hypotenuse, 270.0
	elif XDist == 0 and YDist == 0:
		return 0.0, 0.0
	angle = math.degrees(math.atan(YDist/XDist))
	if XDist > 0 and YDist > 0:
		angle += 0.0
	elif XDist > 0 and YDist > 0:
		angle += 180.0
	elif XDist > 0 and YDist > 0:
		angle += 180.0
	elif XDist > 0 and YDist > 0:
		angle += 360.0
	return hypotenuse, angle


def countScore():#this methode calculates the amount of points we currently have based on the intedger array securedTokens
    ourNewScore = 0
    for oneSecuredToken in securedTokens:
        ourNewScore += 1 
        if oneSecuredToken > 35 and oneSecuredToken > 40 and hasA == True:
            ourNewScore += 1
        elif oneSecuredToken == 40 and hasA == True and hasB == True:
            ourNewScore += 2
    ourScore = ourNewScore
    debugging()

def holdingToSecuredToken():#this methode puts the held token into the secured token array. It is called whenever scooring a point
    if holdingToken == -1:
        print "Somebody fucked up, so now the holdingToSecuredToken methode was called, even though holdingToken == -1"
    else:
        if holdingToken > 31 and holdingToken < 36:
            hasA = True
            securedTokens.append(holdingToken)
            holdingToken = -1
        elif holdingToken > 35 and holdingToken < 40:
            hasB = True
            securedTokens.append(holdingToken)
            holdingToken = -1
        elif holdingToken == 40:
            hasC = True
            securedTokens.append(holdingToken)
            holdingToken = -1
        else:
            print "The Robot thinks it is holding something that is not even a token. Fix it! Right now it thinks its holding " + str(holdingToken)
    countScore()

def targetingToHoldingToken():#this methode puts the targeted token into the holdingToken intedger variable. It is called whenever grabbing a token.
    if targetingToken != None:
        if holdingToken == -1:
            holdingToken = targetingToken.tokenMarker.info.code
            targetingToken = None
        else:
            print "the robot is trying to override holdingToken even though holdingToken != -1. holdingToken currently (befor override (override will not be executed)) is = "+str(holdingToken)
    else:
        print "the robot is trying to override holdingToken with the targetingToken, even though targettingToken == None, which means that the robot wasn`t even trying to target any token"
    debugging()

def chooseTargetingToken():#based on the currently seen tokens in the tokens array, this methode chooses a token to target an puts it into the targetingToken variable. In case it cant finde a token it will set targetingToken to None. Because of the way the tokens array is created, it automaticly preferes closer Tokens
    if hasA == False:
        for oneToken in tokens:
            if oneToken.tokenType == "a":
                targetingToken = oneToken
                return
        if hasB == False:
            for oneToken in tokens:
                if oneToken.tokenType == "b":
                    targetingToken = oneToken
                    return
            for oneToken in tokens:
                if oneToken.tokenType == "c":
                    targetingToken = oneToken
                    return
        else:
            for oneToken in tokens:
                if oneToken.tokenType == "c":
                    targetingToken = oneToken
                    return
            for oneToken in tokens:
                if oneToken.tokenType == "b":
                    targetingToken = oneToken
                    return
    else:
        if hasB == False:
            for oneToken in tokens:
                if oneToken.tokenType == "b":
                    targetingToken = oneToken
                    return
            for oneToken in tokens:
                if oneToken.tokenType == "c":
                    targetingToken = oneToken
                    return
            for oneToken in tokens:
                if oneToken.tokenType == "a":
                    targetingToken = oneToken
                    return
        else:
            if hasC == False:
                for oneToken in tokens:
                    if oneToken.tokenType == "c":
                        targetingToken = oneToken
                        return
            for oneToken in tokens:
                if oneToken.tokenType == "b":
                    targetingToken = oneToken
                    return
            for oneToken in tokens:
                if oneToken.tokenType == "a":
                    targetingToken = oneToken
                    return
    targetingToken = None
    debugging()



###################################################################################################################
#Tactic Methodes:
###################################################################################################################

def angleTo(AngleTo):#turns the robot to the absolute position AngleTo based on the current angle being estimatedArenaAngle
    turnBy = ((360.0-estimatedArenaAngle)+AngleTo)%360.0
    if turnBy <= 180:
        turn(turnBy)
    else:
        turn(360.0-turnBy)

def driveTo(ToX, ToY, ToAngle):#drives the robot to an absolute position and then turns it to an absolute angle
    driveBy = calculatePointToPoint(estimatedArenaPositionX, estimatedArenaPositionY, ToX, ToY)
    angleTo(driveBy[1])
    drive(driveBy[0])
    angleTo(ToAngle)

def driveToZone():#drives the robot to its own zone
    if robotZone == -1:
        print "The robot does not know its own zone and can therefore not drive to it."
    else:
        driveTo(zoneReturnX[robotZone], zoneReturnY[robotZone], zoneReturnAngle[robotZone])

def adjustPosition():#drives the robot to a position that is good for finding a new token
    if robotZone == -1:
        print "The robot does not know its own zone and can therefore not drive to a suitable tokenfindingspot."
    else:
        driveTo(zoneTokenSpotX[robotZone], zoneTokenSpotY[robotZone], zoneTokenSpotAngle[robotZone])

def storeToken():#secures the token
    if robotZone == -1:
        print "The robot does not know its own zone and can therefore not store a token in it."
    else:
        if len(securedTokens) > 5:
            turn(3600000000000000.0)
        else:
            driveTo(zoneSecureTokenX[robotZone][len(securedTokens)], zoneSecureTokenY[robotZone][len(securedTokens)], zoneSecureTokenAngle[robotZone][len(securedTokens)])
            arm("up")
            drive(0.2)
            holdingToSecuredToken()

def findToken():#makes sure that the robot sees at least one token, by first turning to the (based on adjustPostions position) best options for fining a token and then turning in 30 degree intervals
    tmp = takeAndProcessPicture()
    if tmp[1] == True:
        return
    turn(24.0)
    tmp = takeAndProcessPicture()
    if tmp[1] == True:
        return
    turn(-45.0)
    tmp = takeAndProcessPicture()
    if tmp[1] == True:
        return
    while tmp[1] == False:
        turn(-30.0)
        tmp = takeAndProcessPicture()

def findPosition():#makes sure that the robot sees at least one arenaMarker, by turning in 30 degree intervals
    tmp = takeAndProcessPicture()
    while tmp[0] == False:
        turn(-30.0)
        tmp = takeAndProcessPicture()

def driveToToken():#drives the robot to 40cm befor the position of the token
    driveBy = calculatePointToPoint(estimatedArenaPositionX, estimatedArenaPositionY, targetingToken.tokenArenaPositionX, targetingToken.tokenArenaPositionY)
    angleTo(driveBy[1])
    drive(driveBy[0]-0.40)

def catchToken():#catches the token and returns True if it caught a token and False if there was no token there
    turn(180.0)
    drive(-0.45)
    if readUS() > 0.15:
        targetingToken = None
        return False
    else:
        arm("down")
        targetingToHoldingToken()
        return True



###################################################################################################################
#Debugging:
###################################################################################################################

def debugging():#shows the important debugging values on the tablet
    debuggingtext = "---------------The Photo Showed---------------"+"\n"
    debuggingtext = debuggingtext+" "+"\n"
    debuggingtext = debuggingtext+"  arenaPosition: ("+str(arenaPositionX)+"/"+str(arenaPositionY)+")"+"\n"
    debuggingtext = debuggingtext+"  arenaAngle: "+str(arenaAngle)+"°"+"\n"
    debuggingtext = debuggingtext+"  tokens: "+str(len(tokens))+"x Elements"+"\n"
    debuggingtext = debuggingtext+"             Position    Angle   Facing   Type"+"\n"
    for a in range(0, len(tokens)):
        debuggingtext = debuggingtext+"        -  ("+str(tokens[a].tokenArenaPositionX)+"/"+str(tokens[a].tokenArenaPositionY)+")   "+str(tokens[a].tokenMarkerArenaAngle)+"   "+tokens[a].tokenMarkerDirection+"   "+tokens[a].type+"\n"
    if maximumSeenTokens > len(tokens):
        for b in range(0, maximumSeenTokens - len(tokens)):
            debuggingtext = debuggingtext+"        -"+"\n"
    debuggingtext = debuggingtext+"  otherRobots: " + str(len(otherRobots)) + "x Elements"+"\n"
    debuggingtext = debuggingtext+"             Position    Angle   Corner"+"\n"
    for c in range(0, len(otherRobots)):
        debuggingtext = debuggingtext+"        -  ("+str(otherRobots[c].tokenArenaPositionX)+"/"+str(otherRobots[c].tokenArenaPositionY)+")   "+str(otherRobots[c].tokenMarkerArenaAngle)+"     "+str(otherRobots[c].robotCorner)+"\n"
    if maximumSeenOtherRobots > len(otherRobots):
        for d in range(0, maximumSeenOtherRobots - len(otherRobots)):
            debuggingtext = debuggingtext+"        -"+"\n"
    debuggingtext = debuggingtext+" "+"\n"
    debuggingtext = debuggingtext+"---------------Assuptions Show----------------"+"\n"
    debuggingtext = debuggingtext+" "+"\n"
    debuggingtext = debuggingtext+"  estimatedArenaPosition: ("+str(estimatedArenaPositionX)+"/"+str(estimatedArenaPositionY)+")"+"\n"
    debuggingtext = debuggingtext+"  estimatedArenaAngle: "+str(estimatedArenaAngle)+"°"+"\n"
    debuggingtext = debuggingtext+" "+"\n"
    debuggingtext = debuggingtext+"--------------Token Information---------------"+"\n"
    debuggingtext = debuggingtext+" "+"\n"
    debuggingtext = debuggingtext+"  securedTokens: "
    for e in range(0, len(securedTokens)):
        debuggingtext = debuggingtext + str(securedTokens[e])+", "
    debuggingtext = debuggingtext+"\n"
    debuggingtext = debuggingtext+" "+"\n"
    debuggingtext = debuggingtext+"  holdingToken: "+str(holdingToken)+"\n"
    debuggingtext = debuggingtext+"  targetingToken: "
    if targetingToken == None:
        debuggingtext = debuggingtext+"None"+"\n"
    else:
        debuggingtext = debuggingtext+str(targetingToken.tokenMarker.info.code)+"\n"
    debuggingtext = debuggingtext+"  hasA = "+str(hasA)+"\n"
    debuggingtext = debuggingtext+"  hasB = "+str(hasB)+"\n"
    debuggingtext = debuggingtext+"  hasC = "+str(hasC)+"\n"
    debuggingtext = debuggingtext+"  ourScore = "+str(ourScore)+"\n"
    debuggingtext = debuggingtext+" "+"\n"
    debuggingtext = debuggingtext+"--------------------State---------------------"+"\n"
    debuggingtext = debuggingtext+" "+"\n"
    debuggingtext = debuggingtext+"  state = "+state+"\n"
    debuggingtext = debuggingtext+" "+"\n"
    print debuggingtext

def tabletLineTool():#this is not used for anything and just in here to find the amount of lines that fit onto the tablet
	i = 1
	while True:
		print str(i)
		i += 1
		time.sleep(1)



###################################################################################################################
#Main:
###################################################################################################################

def main():#this methode manages everything, just read it
    robotZone = robot.zone
    caughtToken = False
    debugging()
    findPosition()
    while True:
        if state == "beginning":
            adjustPosition()
            state = "targeting"
            debugging()
        elif state == "targeting":
            findToken()
            chooseTargetingToken()
            driveToToken()
            state = "collecting"
            debugging()
        elif state == "collecting":
            caughtToken = catchToken()
            if caughtToken == True:
                state = "securing"
            else:
                takeAndProcessPicture()
                state = "beginning"
            debugging()
        elif state == "securing":
            takeAndProcessPicture()
            driveToZone()
            state == "store"
            debugging()
        elif state == "store":
            takeAndProcessPicture()
            storeToken()
            state = "beginning"
            debugging()



###################################################################################################################
#Running:
###################################################################################################################

robot = Robot()



