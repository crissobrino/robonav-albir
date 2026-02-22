from servos import *
from camera import *
from pid_control import PID
import time, math , random
from tuning import *
import time
from machine import LED


class mapSolver(object):

    def __init__(self,start,orientation,thresholds,gain = 25, p=0.000001, i=0.00000001, d=0,imax = 0.01, p1 = 0.5 , i1=0, d1 =0.0003, p2=0.55, i2=0, d2=0):

        self.cam = Cam(thresholds, gain) #normal stuff
        self.servo = Servo()
        self.dtime = 0
        self.redId  = 1
        self.blueId = 2
        self.greenId = 3
        self.possibleEdge = [0,1,2,4,5,6] #Possible places where the edge could be - this is updated as we move through
        self.PID = PID(p, i, d, imax) #Cris knows what all of these are for
        self.PID1 = PID(p1,i1,d1, imax)
        self.PID2 = PID(p2,i2,d2, imax)
        self.innerMap = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]] #This is the empty map 7x6 where we start in the middle of the 7 and 3 of those 7 will be rows outside of the actual map (edge or beyond)
        self.position = [3,0] # So the robot knows where it is on its internal map- This will always be [3][0] for us not knowing the starting position as Bence explained
        self.innerMap[3][0] = 4 #4s mark places on the internal map that we have been before and hence don't need to revisit
        self.orientation = orientation #Tells us which way we are facing so we know where the things we see should be on the internal map that we build up
        self.finished = False #Keeps track of reaching green so we don't have an infinite loop
        self.times_list = [1.8] #times list for dead angle at edge
        self.scan_direction = 1
        



    def lookRobot(self): #Check the colour of the space we are looking at and store it in correct position of the internal map
        x = self.position[0] #rename these for ease of notation
        y = self.position[1]
        direction = self.orientation
        try: #This will trigger (go to the except part) if we are beyond the edges of the map (the 7x6 not the real 4x6)
            colour = self.checkColour() #Obtain the colour of the blob that we can see
            #Below is the general code which converts the 8 directions into coordinates on the map based on the robots current position
            self.innerMap[(x-int(1<=direction<=3)+int(5<=direction<=7))%10][(y+int(0<=direction<=1 or direction ==7)-int(3<=direction<=5))%10] = colour #Put colour onto internal map
            if colour == 3: #If we see the green just go straight there
                self.driveRobot()
                self.finished = True
            elif self.possibleEdge != []: #If we still don't know where the edges are
                if colour == 2 and (x-int(1<=direction<=3)+int(5<=direction<=7))%10 in self.possibleEdge: #If we have just found that a row can't be an edge because we see blobs in that row
                    self.possibleEdge.remove((x-int(1<=direction<=3)+int(5<=direction<=7))%10) #No longer consider that a possible row
                    try:
                        self.possibleEdge.remove((x+4-int(1<=direction<=3)+int(5<=direction<=7))%8) #YES THESE ARE SUPPOSED TO BE %8 NOT %10
                        self.innerMap[(x+4-int(1<=direction<=3)+int(5<=direction<=7))%8]= [5 for i in range(6)]#Update map to say that the row 4 away must be outside of the map due to the known width (this wraps around and works in both directions using the %)
                    except (ValueError,IndexError):
                        pass
                elif colour == 5: #if we don't see a blob we decide that row is an edge
                    try:
                        self.innerMap[(x-int(1<=direction<=3)+int(5<=direction<=7))%10] = [5 for i in range(6)] #fill in edge and matching one on the other side
                    except IndexError:
                        pass
                    try:
                        self.innerMap[(x+5-int(1<=direction<=3)+int(5<=direction<=7))%10] = [5 for i in range(6)]
                    except IndexError:
                        pass
                    self.possibleEdge = []
                    return True #This tells us that we just discovered the edge which will help avoid turning where we have no blobs to guide us
        except IndexError:
            print("Looking beyond map bounds") #If looking outside the map
            if (y+int(0<=direction<=1 or direction ==7)-int(3<=direction<=5))%10<6: #if we get to the edge of the x direction make the appropriate rows into edges
                for i in range(6):
                        self.innerMap[(x-int(1<=direction<=3)+int(5<=direction<=7))%7][i] = 5
                        self.innerMap[(x+5-int(1<=direction<=3)+int(5<=direction<=7))%7][i] = 5
                        self.possibleEdge = []
        return False #We haven't found an edge this time

    def checkEnclosed(self): #This can remove squares from the map that we don't want to visit because they don't give us new information - we can still pass through if they are on the way to the goal
            for x,k in enumerate(self.innerMap):
                for y,j in enumerate(k):
                    if j ==2:
                        for i in range(8):
                            try:
                                if self.innerMap[(x-int(1<=i<=3)+int(5<=i<=7))%10][(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%10] == 0: #Only blue squares which border unknown squares are useful
                                    break #(Stops the else from happening)
                            except IndexError:
                                pass
                        else: #Changes blue to 4 if there are no unknowns around it
                            self.innerMap[x][y] =4
    def unenclosed(self):
        for x,k in enumerate(self.innerMap):
            for y,j in enumerate(k):
                if j ==4:
                    for i in range(8):
                        try:
                            if self.innerMap[(x-int(1<=i<=3)+int(5<=i<=7))%10][(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%10] == 0: #Only blue squares which border unknown squares are useful
                                break #(Stops the else from happening)
                        except IndexError:
                            pass
                    else: #Changes blue to 4 if there are no unknowns around it
                        self.innerMap[x][y] = 2

    def checkColour(self): #Gets the biggest blob's colour - should be the one in the space we are looking at
            blobs, img = self.cam.get_blobs(self.servo.pan_pos)
            sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

            if sorted_blobs != []:
                cx = sorted_blobs[0].cx()
                cy = sorted_blobs[0].cy()
                print("cy = ", cy)
                if cy < int(sensor.height()/5) or cx < int(sensor.height()/5) and cx > int(4*sensor.height()/5):
                    try:
                        target_blob =  sorted_blobs[1]
                    except:
                        print("no second blob")
                        return 1
                else:
                    target_blob = sorted_blobs[0]

            print(blobs)
            if blobs == []: #If we don't see a blob then we assume this is the edge
                return 5
            else:
                return math.floor(math.log(int(self.cam.get_blob_colours([target_blob])[0]),2)) #Converts the binary blob colour into the colour definitions used in the internal map

    def exploreRobot(self): #The general searching function - checks the local squares which haven't been seen before - drives to the adjacent free square with highest priority - if in a dead end drive to the highest priority blue space in memory
        edgeDiscovered = True #If we find an edge
        while edgeDiscovered: #If we find a new edge we need to restart this process to avoid turning blindly
            x = self.position[0]
            y = self.position[1]
            unseen = [] #Squares we care about checking because we don't know their colour
            edgesP = [] #The position (relative directions out of the 8) of any adjacent edge spaces
            self.reversed = False
            for i in range(8): #Check all directions for unknown and edge spaces
                try:
                    space = self.innerMap[(x-int(1<=i<=3)+int(5<=i<=7))%10][(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%10]
                    if  space == 0:
                        unseen += [i]
                        print("unseen = ", unseen)
                    elif space == 5:
                        edgesP += [i]
                except IndexError:
                    edgesP+=[i]
            self.edges = list(map(self.turnAngles,edgesP)) #Checks where the edge is relative to where we are looking
            if 0 in self.edges: #If we are looking at an edge
                if 1 in self.edges and -1 in self.edges: #If there are edges in both 45 deg locations we need to reverse to the previous location
                    self.reverseRobot()
                    self.reversed = True #This function DEFINITELY needs tuning - might also need further refinement - likely needs to break out of this loop
                elif 2 in self.edges: #We are looking at the right side of an edge so turn back to right
                    self.turn45(-1)
                elif -2 in self.edges:#We are looking at the left side- turn left
                    self.turn45(1)
                self.edges = list(map(self.turnAngles,edgesP))#Update where the edges are relative to direction we are looking at

            if not self.reversed:
                turns = list(map(self.turnAngles,unseen)) #Check how much we need to turn to see each unknown space
                if self.edges != []: #If there are edges to deal with
                    optTurns = list(map(self.optTurnAngles,turns)) #Only turn in the direction where we won't go past the edge
                else:
                    optTurns = turns
                left = [0] + sorted([t for t in optTurns if t>0]) #Left turns are positive
                right = [0] + sorted([t for t in optTurns if t<0],reverse = True) #Right turns negative
                turns= [0]+[left[idx]-left[idx-1] for idx in range(1,len(left))]+[-left[-1]]+[right[idx]-right[idx-1] for idx in range(1,len(right))]
                print(turns)
                time.sleep(2)
                for turn in turns: #List of turns relative to the previous turn (if you need to see 1 and 2 you need to turn 1 and then 1 again...)
                    print("turn:", turn)
                    self.turn45(turn)

                    edgeDiscovered = self.lookRobot() #Did we find the edge
                    if edgeDiscovered == True:
                        break #Don't turn into the edge which we have just discovered

        self.checkEnclosed() #Clean the map with new info
        fullMap = [2 in iter for iter in self.innerMap]
        if not True in fullMap:
            self.unenclosed()
        if not self.finished: #If we didn't just find and drive to the green - can maybe put this in earlier parts too to avoid spinning on the goal needlessly
            print("in not finished loop")
            for i in [1,7,0,2,6,3,5,4]: #Pick priority for blue squares to explore
                try:
                    colour = self.innerMap[(x-int(1<=i<=3)+int(5<=i<=7))%10][(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%10]
                    if colour ==2: #drive to the first blue square in the priority
                        self.getAllEdges()
                        self.edges = list(map(self.turnAngles,self.edges))
                        self.turn45(self.optTurnAngles(self.turnAngles(i)))
                        self.driveRobot() #Drive straight forward to the next square - also updates position of the bot on its internal map
                        print("before break")
                        break #Once we move we don't need any
                except IndexError:
                    print("indexerror")
                    pass
            else: #If we are trapped - there are no blue squares in the adjacent area which we haven't already visited
                path = self.pseudoStar(self.position,self.innerMap,prev = [self.position]) #Finds the shortest prioritised path which will take you to a new blue square by tracking back over previous positions
                #print(path)
                if path == None: #If we don't know any blue squares to go to - something has gone wrong
                    #print(self.position)
                    pass

                else:
                    for i in path: #Travel the path to the blue that we just calculated
                        turn = self.turnAngles(i) #Turn from current orientation to next on path - needs safeguards against being stuck in the edge (no guidance for turns)
                        self.getAllEdges()
                        if self.edges != []:
                            edges = self.turnAngles
                            turn = self.optTurnAngles(turn)
                        self.turn45(turn)
                        self.drive()#TODO

    def turnAngles(self, direction): #number of turns to get from current orientation to desired - not optimised fully
        return((direction - self.orientation))

    def getAllEdges(self):
        self.edges = []
        for i in range(8):
            try:
                if self.innerMap[(self.position[0]-int(1<=i<=3)+int(5<=i<=7))%10][(self.position[1]+int(0<=i<=1 or i ==7)-int(3<=i<=5))%10]==0:
                    self.edges += [i]
            except IndexError:
                self.edges += [i]
        return self.edges

    def optTurnAngles(self,turn):
        st = self.sign(turn)
        se = self.sign(self.edges[1])
        if st==1 and se==1:
            if turn>self.edges[1]:
                return turn-8
        elif st ==-1 and se==-1:
            if turn<self.edges[1]:
                return turn +8
        return turn

    def sign(self, x):
        return (-1)**int(x<0)


    def turn45(self, reps: int) -> None: #Cris needs to explain the next couple
        print("turn45, reps: ", reps)
        if reps>0:
            angle = 55

        elif reps <0:
            angle = -60
        else:
            print("reps=0")
        for i in range(abs(reps)):

            if self.scan_for_blob():
                print("Turning (with blob)")
                self.center_robot(angle)

                self.servo.set_angle(0)
                time.sleep(2)
                continue

            else:
                print("Turning (no blob)")
                self.servo.set_angle(-angle)
                self.center_robot(0)
                print(self.innerMap)
                time.sleep(2)
                continue
        self.orientation = (self.orientation+reps)%8
        print(self.innerMap)

    def scan_for_blob(self, step = 1, limit = 15) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        start_time = time.time()
        start_angle = self.servo.pan_pos
        while True:


            blobs, img = self.cam.get_blobs()
            sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

            # Check blobs to see if reference blob is found
            if sorted_blobs != []:
                # Top boundary of the bottom 4/5
                # x, y, w, h = sorted_blobs[0].rect()  # Get bounding box of the blob
                # if y + h > self.bottom_fifth:  # Check if any part of the blob is in the bottom 4/5
                target_blob = sorted_blobs[0]
                return True
                # else:
                #     try:
                #         target_blob = sorted_blobs[1]
                #         return True
                #     except:
                #         continue

            # Set new angle

            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)
            self.servo.set_angle(new_pan_angle)
            # Update pan angle based on the scan direction and speed
            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= start_angle+limit or self.servo.pan_pos <= start_angle-limit:
                self.scan_direction *= -1

            if time.time() - start_time > 3:
                print("No blob found")
                return False


    def center_robot(self, target_angle: int = 0, limit: int = 15) -> None:
        centered_cam = False

        while True:
            blobs, img = self.cam.get_blobs(self.servo.pan_pos)
            sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)
            # Phase 1: center pan
            if sorted_blobs != []:
                cx = sorted_blobs[0].cx()
                cy = sorted_blobs[0].cy()
                print("cy = ", cy)
                if cy < int(sensor.height()/8) or cx < int(sensor.height()/7) and cx > int(4*sensor.height()/7):
                    try:
                        target_blob =  sorted_blobs[1]
                    except:
                        print("no further or lateral blobs")
                        continue
                else:
                    target_blob = sorted_blobs[0]
                errorObst = -target_blob.cx() + self.cam.w_centre
                angle_error = -(errorObst/sensor.width()*self.cam.h_fov)
                self.servo.set_angle(self.servo.pan_pos - self.PID2.get_pid(angle_error,1))
                if errorObst < 130 and errorObst > -130:
                    centered_cam = True
            else:
                self.scan_for_blob(limit = 8)

            # Phase 2: center robot
            if centered_cam == True:
                angle_error = target_angle - self.servo.pan_pos
                # print("target_angle", target_angle) # target_angle > 0 right, < 0 left
                # print("servo angle", self.servo.pan_pos)
                # print("angle_error", angle_error)

                if abs(angle_error) > 2:
                    if target_angle > 0:
                        if angle_error > 0:
                            self.servo.set_speed(-0.7,0.7)
                        else:
                            self.servo.set_speed(0.7, -0.7)
                    else:
                        if angle_error > 0:
                            self.servo.set_speed(-0.7,0.7)
                        else:
                            self.servo.set_speed(0.7,-0.7)

                    time.sleep(0.01)
                    self.servo.set_differential_drive(0,0)
                    centered_cam = False

                else:
                    self.servo.set_differential_drive(0,0)
                    print("done")
                    break



    def reverseRobot(self): #When the robot is stuck on an edge and needs to reverse to an easier spot - function is placeholder hardcode
        self.servo.set_speed(-0.7,-0.7)
        time.sleep(0.3)
        self.servo.set_differential_drive(0,0)
        i = self.orientation
        self.position = [(self.position[0]+int(1<=i<=3)-int(5<=i<=7))%10,(self.position[1]-int(0<=i<=1 or i ==7)+int(3<=i<=5))%10]
        if not 0<=self.position[0]<=6 and not 0<=self.position[1]<=5:
            print('Escaped')


    def driveRobot(self, speed: float = 0.1) -> None: 
        # Driving forwards to the next blob
        edge_time = 0.145 # to cover the dead angle when the robot doesnt see the blob anymore and has to go forward blindly
        start_time = 0
        end_time = 0
        
        # Update position before driving
        x = self.position[0]
        y = self.position[1]
        direction = self.orientation
        self.position = [(x-int(1<=direction<=3)+int(5<=direction<=7))%10,(y+int(0<=direction<=1 or direction ==7)-int(3<=direction<=5))%10]
        self.innerMap[self.position[0]][self.position[1]] = 4

        # PHASE 1: Go forward until the next blob is at the bottom of the camera 
        while True:
            #look for blobs and figure out if the target blue blob is at the bottom
            all_blobs, img1 = self.cam.get_blobs()
            sorted_blobs = sorted(all_blobs, key=lambda b: b.pixels(), reverse=True)
            bottom_blobs, img2 = self.cam.get_blobs_bottom()
            found_bottom_blue = self.cam.find_blob(bottom_blobs, self.blueId)
            found_bottom_green = self.cam.find_blob(bottom_blobs, self.greenId)
            found_blue = self.cam.find_blob(sorted_blobs, self.blueId)
            found_green = self.cam.find_blob(sorted_blobs, self.greenId)
            self.center_robot(0,0)
            print(found_blue)

            if found_bottom_blue != None or found_bottom_green != None:
                print("Saw bottom blob - entering phase 2")
                break

            else: # drive following next blue/red blob
                if found_blue != None:
                    self.follow_blob_robot(found_blue, sorted_blobs, speed, self.blueId)
                    bottom_blobs = None
                    found_blue = None
                    found_green = None
                elif found_green != None:
                    self.follow_blob_robot(found_green, sorted_blobs, speed, self.greenId)
                    bottom_blobs = None
                    found_green = None
                    found_blue = None

        self.servo.set_differential_drive(0,0)
        time.sleep(0.5)
        # # PHASE 2: When the target blob is getting close (at the bottom of the screen), go forward until it dissapears
        '''
        This is because when the robot won't see this blob anymore it needs to go forward until the next blob is at the bottom of the screen
        The robot will need to know wether the blob that it is seeing at the bottom of the screen is the first target blob or the next one 
        The way we do this is by saying "i already saw a blob in the bottom so the next blob i see is not the target "
        '''

        start_time = time.time()
        while True:
            #look for blobs and figure out if the target blue blob is at the bottom - bottom coords tuned in camera
            all_blobs, img1 = self.cam.get_blobs(self.servo.pan_pos)
            sorted_blobs = sorted(all_blobs, key=lambda b: b.pixels(), reverse=True)
            bottom_blobs, img2 = self.cam.get_blobs_bottom()
            found_bottom_blue = self.cam.find_blob(bottom_blobs, self.blueId)
            found_bottom_green = self.cam.find_blob(bottom_blobs, self.greenId)
            found_blue = self.cam.find_blob(sorted_blobs, self.blueId)
            found_green = self.cam.find_blob(sorted_blobs, self.greenId)
            self.center_robot(0,0)

            print("phase2")

            if found_bottom_blue == None and found_bottom_green == None:
                print("Stopped seeing bottom blob - entering phase 3")
                break

             # drive following next blue blob
            elif found_blue != None:
                self.follow_blob_robot(found_blue, sorted_blobs, speed, self.blueId)
                bottom_blobs = None
                found_blue = None
                found_green = None
            elif found_green != None:
                self.follow_blob_robot(found_green, sorted_blobs, speed, self.greenId)
                bottom_blobs = None
                found_green = None
                found_blue = None
            
            #we are now operating in the dead angle
            #this code is to make the robot go forward a bit once it stops seeing the target blob by using the next blob to align itself
            #when we are at the edge we cannot see the next blob so in stead we use time 
            #the time that the robot takes from the moment when it stops seeing the blob and when is at the center is the time of phase 3

            end_time = time.time()
            self.times_list.append(start_time - end_time)
            if self.times_list:  # Ensure the list is not empty to avoid division by zero
                edge_time = sum(self.times_list) / len(self.times_list)

        # PHASE 3: Operating blind, the target blob disapeared but we are not yet at the center
        self.servo.set_differential_drive(0,0)
        time.sleep(0.5)
        while True:
            #look for blobs and figure out if the target blue blob is at the bottom
            all_blobs, img1 = self.cam.get_blobs()
            sorted_blobs = sorted(all_blobs, key=lambda b: b.pixels(), reverse=True)
            bottom_blobs, img2 = self.cam.get_blobs_bottom()
            found_bottom = self.cam.find_blob(bottom_blobs, self.blueId)
            found_blue = self.cam.find_blob(sorted_blobs, self.blueId)
            found_red = self.cam.find_blob(sorted_blobs, self.redId)
            found_green = self.cam.find_blob(sorted_blobs, self.greenId)


            if sorted_blobs != []:
                cx = sorted_blobs[0].cx()
                cy = sorted_blobs[0].cy()
                print("cy = ", cy)
                # filter out further red blobs that are bigger than target blobs and blobs at the sides
                if cy < int(sensor.height()/8) or cx < int(sensor.height()/5) and cx > int(4*sensor.height()/5):
                    try:
                        stopper_blob =  sorted_blobs[1]
                    except:
                        print("Only further or lateral blobs")
                        continue
                else:
                    stopper_blob = sorted_blobs[0]
            else:
                continue

            if self.check_edge() == False:
                # stopper blob limits in camera depending on whether we're driving straight or diagonal
                self.center_robot(0,0)
                if self.orientation%2 == 0:
                    top_limit = 180
                    bottom_limit = 190
                else:
                    top_limit= 110
                    bottom_limit = 130
                x, y, w, h = stopper_blob.rect()  # Get bounding box of the blob
                bottom_centre = stopper_blob.cy()+h/2
                print(bottom_centre)
                if (bottom_centre > top_limit and bottom_centre < bottom_limit) :  # Check if any part of the blob is in the bottom half
                    print("Stopper blob detected")
                    self.servo.set_speed(0,0)
                    time.sleep(2)
                    break

                else:  #drive following next blob
                    bias = 0
                    if found_blue != None or found_red != None or found_green!=None:
                        next_blob = stopper_blob
                        errorMid = next_blob.cx() - self.cam.w_centre
                        self.bias = self.PID1.get_pid(errorMid,1)
                        print("driving")
                    self.servo.set_differential_drive(speed,bias)
                    

            else:
                self.servo.set_differential_drive(speed,0)
                time.sleep(edge_time)
                self.servo.set_speed(0,0)
                time.sleep(1)
                break



    def driveRobot_blind(self, speed: float = 0.3) -> None: 
        #Same as drive robot but for version with no map - cannot know when it's at an edge
        edge_time = 0.145
        saw_bottom_blob_1 = False
        saw_bottom_blob_2 = False
        bottom_blob_1_dissapeared = False
        start_time = 0
        end_time = 0
        check_edge = False


        # Phase 1
        while True:
            all_blobs, img1 = self.cam.get_blobs()
            sorted_blobs = sorted(all_blobs, key=lambda b: b.pixels(), reverse=True)
            bottom_blobs, img2 = self.cam.get_blobs_bottom()
            found_bottom_blue = self.cam.find_blob(bottom_blobs, self.blueId)
            found_bottom_green = self.cam.find_blob(bottom_blobs, self.greenId)
            found_blue = self.cam.find_blob(sorted_blobs, self.blueId)
            found_green = self.cam.find_blob(sorted_blobs, self.greenId)
            self.center_robot_blue(0,0)
            print(found_blue)

            if found_bottom_blue != None or found_bottom_green != None:
                print("Saw bottom blob - entering phase 2")
                break

            # drive following next blue or green blob
            else:
                if found_blue != None:
                    self.follow_blob_robot(found_blue, sorted_blobs, speed, self.blueId)
                    bottom_blobs = None
                    found_blue = None
                    found_green = None
                elif found_green != None:
                    self.follow_blob_robot(found_green, sorted_blobs, speed, self.greenId)
                    bottom_blobs = None
                    found_green = None
                    found_blue = None

        self.servo.set_differential_drive(0,0)
        time.sleep(0.5)
        # Phase 2
        start_time = time.time()
        while True:
            all_blobs, img1 = self.cam.get_blobs(self.servo.pan_pos)
            sorted_blobs = sorted(all_blobs, key=lambda b: b.pixels(), reverse=True)
            bottom_blobs, img2 = self.cam.get_blobs_bottom()
            found_bottom_blue = self.cam.find_blob(bottom_blobs, self.blueId)
            found_bottom_green = self.cam.find_blob(bottom_blobs, self.greenId)
            found_blue = self.cam.find_blob(sorted_blobs, self.blueId)
            found_green = self.cam.find_blob(sorted_blobs, self.greenId)
            self.center_robot_blue(0,0)

            print("phase2")

            if found_bottom_blue == None and found_bottom_green == None:
                print("Stopped seeing bottom blob - entering phase 3")
                break

             # drive following next blue blob
            elif found_blue != None:
                self.follow_blob_robot(found_blue, sorted_blobs, speed, self.blueId)
                bottom_blobs = None
                found_blue = None
                found_green = None
            elif found_green != None:
                self.follow_blob_robot(found_green, sorted_blobs, speed, self.greenId)
                bottom_blobs = None
                found_green = None
                found_blue = None

            end_time = time.time()
            self.times_list.append(start_time - end_time)
            if self.times_list:  # Ensure the list is not empty to avoid division by zero
                edge_time = sum(self.times_list) / len(self.times_list)

        # Phase 3
        self.servo.set_differential_drive(0,0)
        time.sleep(0.5)
        while True:
            all_blobs, img1 = self.cam.get_blobs()
            sorted_blobs = sorted(all_blobs, key=lambda b: b.pixels(), reverse=True)
            bottom_blobs, img2 = self.cam.get_blobs_bottom()
            found_bottom = self.cam.find_blob(bottom_blobs, self.blueId)
            found_blue = self.cam.find_blob(sorted_blobs, self.blueId)
            found_red = self.cam.find_blob(sorted_blobs, self.redId)
            found_green = self.cam.find_blob(sorted_blobs, self.greenId)


            if sorted_blobs != []:
                cx = sorted_blobs[0].cx()
                cy = sorted_blobs[0].cy()
                print("cy = ", cy)
                if cy < int(sensor.height()/8) or cx < int(sensor.height()/5) and cx > int(4*sensor.height()/5):
                    try:
                        stopper_blob =  sorted_blobs[1]
                    except:
                        print("no further or lateral blobs")
                        continue
                else:
                    stopper_blob = sorted_blobs[0]
            else:
                continue

            if check_edge== True:
                self.center_robot(0,0)
                if self.orientation%2 == 0:
                    top_limit = 180
                    bottom_limit = 190
                else:
                    top_limit= 110
                    bottom_limit = 130
                x, y, w, h = stopper_blob.rect()  # Get bounding box of the blob
                bottom_centre = stopper_blob.cy()+h/2
                print(bottom_centre)
                if (bottom_centre > top_limit and bottom_centre < bottom_limit) :  # Check if any part of the blob is in the bottom half
                    print("Stopper blob detected")
                    self.servo.set_speed(0,0)
                    time.sleep(2)
                    break

                else:  #drive following next blue blob
                    bias = 0
                    if found_blue != None or found_red != None or found_green!=None:
                        next_blob = stopper_blob
                        errorMid = next_blob.cx() - self.cam.w_centre
                        self.bias = self.PID1.get_pid(errorMid,1)
                        print("driving")
                    self.servo.set_differential_drive(0.7,0)
                    time.sleep(0.08)
                    self.servo.set_differential_drive(0,0)
                    time.sleep(0.2)

            else:
                self.servo.set_differential_drive(speed,0)
                time.sleep(edge_time)
                self.servo.set_speed(0,0)
                time.sleep(1)
                break



    def follow_blob_robot(self, found_colour, blobs_list, speed, blobId) -> None:
        bias = 0
        if found_colour != None:
            next_blue_blob = blobs_list[self.cam.find_blob(blobs_list, blobId)]
            errorMid = next_blue_blob.cx() - self.cam.w_centre
            self.bias = self.PID1.get_pid(errorMid,1)
        print("driving")
        self.servo.set_differential_drive(speed,bias)

    


    def check_edge(self): #Checks if we are at the edge of the 7x6 map - needs to also check if the space is marked 5.
            try:
                i = self.orientation
                colour = self.innerMap[(self.position[0]-int(1<=i<=3)+int(5<=i<=7))%10][(self.position[1]+int(0<=i<=1 or i ==7)-int(3<=i<=5))%10]
            except IndexError:
                self.edge = True
                return self.edge
            if colour == 5:
                self.edge = True
            else:
                self.edge = False
            return self.edge

    def pseudoStar(self,position, map, safe = 4, goal = 2, prev = [],testing = False): #Recursive pathfinding algorithm - pretty much always used with default arguments for main usecase
        x = position[0] #Important to note that this is not the position of the robot but a position of a virtual robot used to find the optimal path
        y = position[1]
        shortest = 1000 #Stores the length of the best path - low is better - this initial value will be beaten by all possible paths which find a useful space - could be swapped out for a different scoring function for the path
        bestPath = None #Keep track of the optimal path
        for i in [1,7,0,2,6,3,5,4]: #Loop through in priority
                space = [(x-int(1<=i<=3)+int(5<=i<=7))%10,(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%10]
                if space not in prev: #If this virtual robot hasn't been to this space before - avoids infinite loops
                    try:
                        colour = map[space[0]][space[1]]
                        if colour == safe: #If the space is a blue space which holds no new information but can be used to reach the goal
                            path = self.pseudoStar(space,map,safe,goal,prev + [space],testing=testing) #We recursively call the function to simulate further virtual robot branches
                            if path != None: #If at least one of the virtual robot branches we sent returned a usable path
                                if len(path) <= shortest: #If it is fewer steps from us than the current lowest step path
                                    try:
                                        if self.innerMap[space[0]-int(1<=i<=3)+int(5<=i<=7)][space[1]+int(0<=i<=1 or i ==7)-int(3<=i<=5)] == 5 and not i%2:
                                            print("Would be looking out the side")
                                        else:
                                            shortest = len(path)
                                            bestPath = [i] + path #Store the best virtual path from this space and the way to reach this space - full path from [x,y] in this iteration of the function
                                    except IndexError:
                                        if not i%2:#TODO
                                            shortest = len(path)
                                            bestPath = [i] + path
                        elif colour == goal:#If we find the colour we are looking for - blue space we haven't visited yet adjacent to the virtual position
                            return([i]) #The last turn in the path to the blue space
                    except IndexError:
                        pass
                elif testing:
                    print(prev)
        return bestPath #Returns best scoring path - currently scored by length

    def blindSolver(self):
        direction = 1
        buffer = False
        justDrove = False
        while not self.finished:
            colour = self.checkColour()

            if justDrove:
                if buffer:
                    justDrove = False
                    buffer = False
                else:
                    buffer = True
            print(colour)
            if colour == 3:
                print("seeing yellow")
                self.driveRobot_blind()
                self.finished = True
            elif colour == 2:
                self.driveRobot_blind()
                justDrove = True
                buffer = False
            elif colour == 1:
                self.turn45(direction)
            else:
                if justDrove == True:
                    self.turnBlind()
                    justDrove == False
                else:
                    direction *= -1
                    self.turn45(direction)

    def turnBlind(self):

        self.servo.set_speed(0.7,-0.7)
        time.sleep_ms(random.randint(300,700))
        self.servo.set_speed(0,0)

# score = 0
# for i in range(1):
#     mapper = mapSolver([random.randint(1,3),0])
#     j  = 0

#     # print(mapper.pseudoStar([0,0],mapper.hiddenMap,2,3))

#     while not mapper.finished and j<100:
#         mapper.explore()

#         j+=1
#     score+=j
#     print(j)

# print(score)
if __name__ == "__main__":
    thresholds = [  ( 0 , 0,  0,  0,   0,   0), #placeholder so the numbers work out - do not change order
                    (14, 21, 19, 41, -24, 44),#Red
                   (11, 25, -21, 0, -15, -1),#Blue
                    (19, 39, -26, -12, 11, 34), #Green
                ]
    solver = mapSolver([3,0],0,thresholds,10)
    solver.servo.soft_reset()
    led = LED("LED_BLUE")
    led.off()
    blobs,img=solver.cam.get_blobs()
    # while True:
    #     time.sleep(0.5)
    #     blobs,img=solver.cam.get_blobs()
    #     solver.servo.set_angle(solver.servo.pan_pos+5)
    #     print(solver.servo.pan_pos)
    # time.sleep(3)
    #print(solver.check_edge())
    # solver.lookRobot()
    # solver.center_robot(0.7)
    # solver.go_forwards_robot(0.1)
    # solver.center_robot(0.7)
    # solver.go_forwards_robot(0.1)
    # solver.center_robot(0.7)
    # solver.driveRobot(0.1)
    # print(solver.innerMap)
    # print(solver.times_list)
    # solver.turn90(1)
    # solver.driveRobot(0.1)
    # solver.turn45(2)
    # # # solver.orientation = 1
    while not solver.finished:           #THIS IS THE WAY TO MAKE THE BOT EXPLORE UNTIL IT REACHES THE END WITHOUT INFINITE LOOP
        solver.exploreRobot()
        # solver.blindSolver()

    print(solver.innerMap)
