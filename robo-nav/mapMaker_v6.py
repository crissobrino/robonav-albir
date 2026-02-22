#from servos import *
from camera import *
from pid_control import PID
import time, math , random
from tuning import *
import time

class mapSolver(object):

    def __init__(self,start,orientation,thresholds,gain = 25, p=0.000001, i=0.000005, d=0,imax = 0.01, p1 = 0.003 , i1=0.4, d1=0.0003, p2=0.15, i2=0, d2=0):

        self.cam = Cam(thresholds, gain)
        self.servo = Servo()
        self.dtime = 0
        self.redId  = 1
        self.blueId = 2
        self.greenId = 3
        self.PID = PID(p, i, d, imax)
        self.PID1 = PID(p1,i1,d1, imax)
        self.PID2 = PID(p2,i2,d2, imax)
        self.innerMap = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
        self.position = start
        self.innerMap[start[0]][start[1]] = 4
        self.orientation = orientation
        possible = None
        while possible == None:
            self.hiddenMap = [[random.randint(1,2) for i in range(6)] for i in range(4)] #Simulated Map
            self.hiddenMap[random.randint(1,3)][5] = 3
            self.hiddenMap[start[0]][start[1]] = 2
            possible = self.pseudoStar(self.position,self.hiddenMap,2,3,[self.position])
        self.finished = False
        self.times_list = []


    def seen(self,direction:int,colour:int):
        '''
        Function to track seen colours
        direction  = current facing direction 0 - 7 where 0 is +y direction and increases anticlockwise
        colour = colour of blob detected (1 = red, 2 = blue, 3 = green)
        '''
        x = self.position[0]
        y = self.position[1]
        try:
            self.innerMap[(x-int(1<=direction<=3)+int(5<=direction<=7))%7][(y+int(0<=direction<=1 or direction ==7)-int(3<=direction<=5))%7]  = colour
            if colour == 3:
                self.drive(direction)
                self.finished = True
        except IndexError:
            #print("Looking beyond map bounds")
            pass


    def look(self,direction):
        #Simulation function
        x = self.position[0]
        y = self.position[1]
        try:
            colour = self.hiddenMap[x-int(1<=direction<=3)+int(5<=direction<=7)][y+int(0<=direction<=1 or direction ==7)-int(3<=direction<=5)]
            self.innerMap[(x-int(1<=direction<=3)+int(5<=direction<=7))%7][(y+int(0<=direction<=1 or direction ==7)-int(3<=direction<=5))%7] = colour
            if colour == 3:
                self.drive(direction)
                self.finished = True
        except IndexError:
            #print("Looking beyond map bounds")
            pass

    def lookRobot(self):
        x = self.position[0]
        y = self.position[1]
        direction = self.orientation
        try:
            colour = self.checkColour()
            self.innerMap[(x-int(1<=direction<=3)+int(5<=direction<=7))%7][(y+int(0<=direction<=1 or direction ==7)-int(3<=direction<=5))%7] = colour
            if colour == 3:
                self.driveRobot()
                self.finished = True
        except IndexError:
            print("Looking beyond map bounds")
            pass

    def checkEnclosed(self):
            for x,k in enumerate(self.innerMap):
                for y,j in enumerate(k):
                    if j ==2:
                        for i in range(8):
                            if self.innerMap[(x-int(1<=i<=3)+int(5<=i<=7))%7][(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%7] == 0:
                                break
                        else:
                            self.innerMap[x][y] =4

    def checkColour(self):
            blobs, _ = self.cam.get_blobs()

            if blobs == None:
                return 0
            else:
                color_index = self.cam.get_blob_colours(blobs)
                largest_blob = self.cam.get_biggest_blob(blobs)
                largest_blob_index = blobs.index(largest_blob)
                return math.floor(math.log(int(self.cam.get_blob_colours([self.cam.get_biggest_blob(blobs)])[0]),2))



    def explore(self):
        x = self.position[0]
        y = self.position[1]
        for i in range(8):
            try:
                if self.innerMap[(x-int(1<=i<=3)+int(5<=i<=7))%7][(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%7] == 0:
                    self.look(i)
            except IndexError:
                pass
        if not self.finished:
            for i in range(8):
                try:
                    colour = self.innerMap[(x-int(1<=i<=3)+int(5<=i<=7))%7][(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%7]
                    if colour ==2:
                        self.drive(i)
                        break
                except IndexError:
                    pass
            else:
                path = self.pseudoStar(self.position,self.innerMap,prev = [self.position])
                #print(path)
                if path == None:
                    #print(self.position)
                    pass

                else:
                    for i in path:
                        self.drive(i)

    def exploreRobot(self):
        x = self.position[0]
        y = self.position[1]
        unseen = []
        for i in range(8):
            try:
                if self.innerMap[(x-int(1<=i<=3)+int(5<=i<=7))%7][(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%7] == 0:
                    unseen += [i]
                    print("unseen=", unseen)
            except IndexError:
                pass
        turns = map(self.turnAngles,unseen)
        turns = sorted(turns)
        print("turns=", turns)
        for turn in [turns[0]]+[turns[idx]-turns[idx-1] for idx in range(1,len(turns))]:
            self.turn45(turn)
            self.lookRobot()

        self.checkEnclosed()
        if not self.finished:
            print("in not finished loop")
            for i in [1,7,0,2,6,3,5,4]:
                try:
                    colour = self.innerMap[(x-int(1<=i<=3)+int(5<=i<=7))%7][(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%7]
                    if colour ==2:
                        self.turn45(self.turnAngles(i))
                        self.driveRobot()
                        print("before break")
                        break
                except IndexError:
                    print("indexerror")
                    pass
            else:
                path = self.pseudoStar(self.position,self.innerMap,prev = [self.position])
                #print(path)
                if path == None:
                    #print(self.position)
                    pass

                else:
                    for i in path:
                        self.turn45(self.turnAngles(i))
                        self.driveRobot()

    def turnAngles(self, direction):
        return((direction - self.orientation)%8)


    def center_robot(self, speed: float) -> None:
        extratime = 0
        bottom_fifth = int(sensor.height() / 6)

        while True:
            blobs, img = self.cam.get_blobs()
            sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

            if sorted_blobs != []:
                # Top boundary of the bottom 4/5
                x, y, w, h = sorted_blobs[0].rect()  # Get bounding box of the blob
                if y + h > bottom_fifth:  # Check if any part of the blob is in the bottom 4/5
                    target_blob = sorted_blobs[0]
                else:
                    try:
                        target_blob = sorted_blobs[1]
                    except:
                        continue

                errorTarget =  self.cam.w_centre - target_blob.cx()

                if abs(errorTarget) > 25:
                    correction_time = -self.PID.get_pid(errorTarget,1)
                    if 0<errorTarget:
                        self.servo.set_speed(speed,-speed)
                        extratime += correction_time
                    else:
                        self.servo.set_speed(-speed,speed)
                        extratime -= correction_time

                    time.sleep(abs(correction_time))
                    self.servo.set_differential_drive(0,0) #needed due to high speed

                else:
                    self.servo.set_differential_drive(0,0)
                    print(extratime)
                    self.dtime = extratime
                    break


    def turn45(self, reps: int, time45: float = 0.145) -> None:
        for i in range(reps):
            self.servo.set_speed(-0.7, 0.7)
            time.sleep(time45 + self.dtime)
            print("turning time:", time45 +self.dtime)
            self.servo.set_differential_drive(0,0)
            self.orientation = (self.orientation+1)%8
            if not self.check_edge():
                self.center_robot(0.7)
            time.sleep_ms(100)

    def turn90(self, reps: int) -> None:
        for i in range(reps):
            # turn pan and adjust
            self.servo.set_angle(-70)
            centered_cam = False
            while True:
                print("looking again")
                blobs, img = self.cam.get_blobs()
                sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)
                # Phase 1: center pan
                if sorted_blobs != []:
                    target_blob = sorted_blobs[0]
                    errorObst = -target_blob.cx() + self.cam.w_centre
                    angle_error = -(errorObst/sensor.width()*self.cam.h_fov)
                    self.servo.set_angle(self.servo.pan_pos - self.PID2.get_pid(angle_error,1))
                    print(errorObst)
                    if errorObst < 130 and errorObst > -130:
                        centered_cam = True

                # Phase 2: center robot
                if centered_cam == True:
                    print("centered")
                    angle_error = self.servo.pan_pos
                    print(angle_error)
                    if abs(angle_error) > 3:
                        print("angle error")
                        if 0<angle_error:
                            print("turning left")
                            self.servo.set_speed(0.7,-0.7)

                        else:
                            print("turning right")
                            self.servo.set_speed(-0.7,0.7)
                        time.sleep(0.01)
                        print("stopped turning")
                        self.drive_robot(0,0)
                        centered_cam = False

                    else:
                        self.drive_robot(0,0)
                        print("done")




    def driveRobot(self, speed: float = 0.1) -> None:
        edge_time = 0
        saw_bottom_blob_1 = False
        saw_bottom_blob_2 = False
        bottom_blob_1_dissapeared = False
        start_time = 0
        end_time = 0

        x = self.position[0]
        y = self.position[1]
        direction = self.orientation
        self.position = [(x-int(1<=direction<=3)+int(5<=direction<=7))%7,(y+int(0<=direction<=1 or direction ==7)-int(3<=direction<=5))%7]
        self.innerMap[self.position[0]][self.position[1]] = 4

        # Phase 1
        while True:
            all_blobs, img1 = self.cam.get_blobs()
            sorted_blobs = sorted(all_blobs, key=lambda b: b.pixels(), reverse=True)
            bottom_blobs, img2 = self.cam.get_blobs_bottom()
            found_bottom = self.cam.find_blob(bottom_blobs, self.blueId)
            found_blue = self.cam.find_blob(sorted_blobs, self.blueId)
            self.center_robot(speed)

            if found_bottom != None:
                print("Saw bottom blob - entering phase 2")
                saw_bottom_blob_1 = True
                break

            # drive following next blue/red blob
            else:
                if found_blue != None:
                    self.follow_blob_robot(found_blue, sorted_blobs, speed, self.blueId)
                    bottom_blobs = None
                    found_blue = None


        # Phase 2
        start_time = time.time()
        while True:
            all_blobs, img1 = self.cam.get_blobs()
            sorted_blobs = sorted(all_blobs, key=lambda b: b.pixels(), reverse=True)
            bottom_blobs, img2 = self.cam.get_blobs_bottom()
            found_bottom = self.cam.find_blob(bottom_blobs, self.blueId)
            found_blue = self.cam.find_blob(sorted_blobs, self.blueId)

            print("phase2")
            print(found_bottom)

            if found_bottom == None:
                print("Stopped seeing bottom blob - entering phase 3")
                bottom_blob_1_dissapeared = True
                break

             # drive following next blue blob
            elif found_blue != None:
                self.follow_blob_robot(found_blue, sorted_blobs, speed, self.blueId)
                bottom_blobs = None
                found_blue = None
        end_time = time.time()
        self.times_list.append(start_time - end_time)
        if self.times_list:  # Ensure the list is not empty to avoid division by zero
            edge_time = sum(self.times_list) / len(self.times_list)


        # Phase 3
        while True:
            all_blobs, img1 = self.cam.get_blobs()
            sorted_blobs = sorted(all_blobs, key=lambda b: b.pixels(), reverse=True)
            bottom_blobs, img2 = self.cam.get_blobs_bottom()
            found_bottom = self.cam.find_blob(bottom_blobs, self.blueId)
            found_blue = self.cam.find_blob(sorted_blobs, self.blueId)
            found_red = self.cam.find_blob(sorted_blobs, self.redId)

            if self.check_edge() == False:
                bottom_fifth = int(sensor.height() / 5)  # Top boundary of the bottom 4/5
                x, y, w, h = sorted_blobs[0].rect()  # Get bounding box of the blob
                if y + h > bottom_fifth:  # Check if any part of the blob is in the bottom 4/5
                    print("Blob detected in bottom 4/5 of the image!")
                    saw_bottom_blob_2 = True
                    self.servo.set_speed(0,0)
                    break                #drive following next blue blob
                else:
                    bias = 0
                    if found_blue != None or found_red != None:
                        next_blob = sorted_blobs[0]
                        errorMid = next_blob.cx() - self.cam.w_centre
                        self.bias = self.PID1.get_pid(errorMid,1)
                        print("driving")
                    self.drive_robot(speed,bias)


            else:
                self.drive_robot(speed,0)
                time.sleep(edge_time)
                self.servo.set_speed(0,0)
                break






    def follow_blob_robot(self, found_colour, blobs_list, speed, blobId) -> None:
        bias = 0
        if found_colour != None:
            next_blue_blob = blobs_list[self.cam.find_blob(blobs_list, blobId)]
            errorMid = next_blue_blob.cx() - self.cam.w_centre
            self.bias = self.PID1.get_pid(errorMid,1)
        print("driving")
        self.drive_robot(speed,bias)

    def drive_robot(self, drive: float, steering: float) -> None:
            """
            Differential drive function for the robot.

            Args:
                drive (float): Speed to set the servos to (-1~1)
                steering (float): Sets the steering to (-1~1)
            """
            # Apply limits
            self.servo.set_differential_drive(drive, steering)

    def drive(self,direction):
        x = self.position[0]
        y = self.position[1]
        self.position = [(x-int(1<=direction<=3)+int(5<=direction<=7))%7,(y+int(0<=direction<=1 or direction ==7)-int(3<=direction<=5))%7]
        self.innerMap[self.position[0]][self.position[1]] = 4


    def check_edge(self):
        i = self.orientation
        self.edge = (self.position[0]-int(1<=i<=3)+int(5<=i<=7))%7>3 or (self.position[1]+int(0<=i<=1 or i ==7)-int(3<=i<=5))%7>6
        return self.edge


    def pseudoStar(self,position, map, safe = 4, goal = 2, prev = [],testing = False):
        x = position[0]
        y = position[1]
        shortest = 1000
        bestPath = None
        for i in range(8):
                space = [(x-int(1<=i<=3)+int(5<=i<=7))%7,(y+int(0<=i<=1 or i ==7)-int(3<=i<=5))%7]
                if space not in prev:
                    try:
                        colour = map[space[0]][space[1]]
                        if colour == safe:
                            path = self.pseudoStar(space,map,safe,goal,prev + [space],testing=testing)
                            if path != None:
                                if len(path) <= shortest:
                                    shortest = len(path)
                                    bestPath = [i] + path
                        elif colour == goal:
                            return([i])
                    except IndexError:
                        pass
                elif testing:
                    print(prev)
        return bestPath


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
                    (15, 33, 25, 44, 9, 33), #Red
                    (20, 33, -17, 5, -31, -9), #Blue
                    (45, 66, 9, 39, 1, 29), #Green
                ]
    solver = mapSolver([3,2],0,thresholds,15)
    solver.servo.soft_reset()
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
    # solver.go_forwards_robot(0.1)

    # print(solver.times_list)
    solver.turn90(1)
    blobs,img=solver.cam.get_blobs()
    # solv
    # solver.exploreRobot()

    print(solver.innerMap)

    # j  = 0
    # while not solver.finished and j<100:
    #     solver.explore()

    #     j+=1
    # solver.exploreRobot()
