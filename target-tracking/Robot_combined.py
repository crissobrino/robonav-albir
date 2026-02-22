from servos import *
from camera import *
from pid_control import PID
import time, math
from tuning import *

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain = 10,
                p=0.4, i=0.25, d=0.15,
                p2=0.3, i2=0.3, d2=0.15,
                p3=1, i3=0.01, d3=0.09 ,
                p4=0.08, i4=0, d4=0.009 ,
                imax=0.01):
        """
        Initializes the Robot object with given PID parameters.

        Args:
            thresholds (list): Colour detection thresholds
            gain (float): Camera gain
            p (float): Proportional gain for the PID.
            i (float): Integral gain for the PID.
            d (float): Derivative gain for the PID.
            imax (float): Maximum Integral error for the PID.
        """
        self.servo = Servo()
        self.servo.soft_reset()
        self.cam = Cam(thresholds, gain)
        self.PID1 = PID(p, i, d, imax)
        self.PID2 = PID(p2, i2, d2, imax)
        self.PID3 = PID(p2,i2,d2)
        self.PID4 = PID(p3,i3,d3)
        self.PID5 = PID(p4,i4,d4)
        self.bias = 0

        # Blob IDs
        self.mid_line_id = 0
        self.obstacle_id = 1
        self.l_line_id = 2
        self.r_line_id = 3

        self.scan_direction = 1

    def stage1(self, speed: float):
        while True:
            blobs, img = self.cam.get_blobs()
            sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

            found_mid = self.cam.find_blob(sorted_blobs, self.mid_line_id)
            found_l = self.cam.find_blob(sorted_blobs, self.l_line_id)
            found_r = self.cam.find_blob(sorted_blobs, self.r_line_id)

            self.biasUpdate(sorted_blobs,found_mid,found_l,found_r,None)
            self.drive(speed,self.bias)


    def stage2(self, speed: float):
        while True:
            blobs, img = self.cam.get_blobs()
            sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

            found_mid = self.cam.find_blob(sorted_blobs, self.mid_line_id)
            found_l = self.cam.find_blob(sorted_blobs, self.l_line_id)
            found_r = self.cam.find_blob(sorted_blobs, self.r_line_id)
            found_obstacle = self.cam.find_blob(sorted_blobs, self.obstacle_id)

            self.biasUpdate(sorted_blobs,found_mid,found_l,found_r,None)
            self.drive(speed,self.bias)

            while found_obstacle !=None:
                self.drive(0,0)
                blobs, img = self.cam.get_blobs()
                sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)
                found_obstacle = self.cam.find_blob(sorted_blobs, self.obstacle_id)



    def stage3(self, speed: float, obstacle_distance: float) -> None:
        """
        Obstacle distance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        mid_line_depth = 29  # in mm
        hidden_distance = 80  # in mm
        frame_bottom_center = (self.cam.w_centre, 480)
        bottom_center_x = None

        # Make sure the robot is facing the mid line
        blobs, img = self.cam.get_blobs()
        time.sleep(0.5)
        found_mid = [blob for blob in blobs if blob[8] == self.mid_line_id + 1]

        largest_mid_line_blob = max(found_mid, key=lambda b: b.pixels())
        mid_line_pixel_depth = largest_mid_line_blob.h()
        scale_factor = mid_line_depth / mid_line_pixel_depth
        print(f"Scale factor: {scale_factor}")

        while True:
            blobs, img = self.cam.get_blobs()
            found_obstacle = [blob for blob in blobs if blob[8] == self.obstacle_id + 1]

            if len(found_obstacle) != 0:
                largest_obstacle_blob = max(found_obstacle, key=lambda b: b.pixels())
                bottom_center_x = largest_obstacle_blob.x() + (largest_obstacle_blob.w() // 2)
                bottom_center_y = largest_obstacle_blob.y() + largest_obstacle_blob.h()

                pixel_distance = math.sqrt((bottom_center_y - frame_bottom_center[1]) ** 2 +
                                      (bottom_center_x - frame_bottom_center[0]) ** 2)

                frame_distance = pixel_distance * scale_factor
                print(f"Frame distance: {frame_distance} mm")

                if 480 - largest_obstacle_blob.h() != 0:
                    frame_angle = abs(math.atan((bottom_center_x - self.cam.w_centre) /
                                  (480 - largest_obstacle_blob.h())))
                    print(f"Angle: {frame_angle * (180 / math.pi)} degrees")
                else:
                    frame_angle = 90

                real_distance = math.sqrt(frame_distance ** 2 + hidden_distance ** 2 -
                                          2 * frame_distance * hidden_distance * math.cos(math.pi - frame_angle))
                print(f"Real distance: {real_distance} mm")


                if real_distance < obstacle_distance * 10:
                    self.drive(0, 0)
                    print("Obstacle detected! Stopping.")
                    break
                else:
                    sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

                    found_mid = self.cam.find_blob(sorted_blobs, self.mid_line_id)
                    found_l = self.cam.find_blob(sorted_blobs, self.l_line_id)
                    found_r = self.cam.find_blob(sorted_blobs, self.r_line_id)

                    self.biasUpdate(sorted_blobs,found_mid,found_l,found_r,None)
                    time.sleep(0.1)

            else:
                print("No obstacle detected! Stopping and searching.")
                sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

                found_mid = self.cam.find_blob(sorted_blobs, self.mid_line_id)
                found_l = self.cam.find_blob(sorted_blobs, self.l_line_id)
                found_r = self.cam.find_blob(sorted_blobs, self.r_line_id)

                self.biasUpdate(sorted_blobs,found_mid,found_l,found_r,None)
                self.drive(speed,self.bias)


        self.servo.soft_reset()
        return

    def stage4(self, speed: float, obstacle_distance: float, turning_angle: float) -> None:
        self.stage3(speed, obstacle_distance)

        centered_cam = False
        while True:
            blobs, img = self.cam.get_blobs()
            sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)
            found_obstacle = self.cam.find_blob(sorted_blobs, self.obstacle_id)

            if found_obstacle != None:
                obstacle_blob = blobs[self.cam.find_blob(blobs, self.obstacle_id)]
                errorObst = -obstacle_blob.cx() + self.cam.w_centre
                angle_error = -(errorObst/sensor.width()*self.cam.h_fov)
                self.servo.set_angle(self.servo.pan_pos - self.PID4.get_pid(angle_error,1))

                if errorObst < 100 and errorObst > -100:
                    centered_cam = True

            if centered_cam == True:
                print("centered")
                angle_error = self.servo.pan_pos - turning_angle
                print(angle_error)
                if abs(angle_error) > 3:
                    if 0<angle_error:
                        self.servo.set_speed(0.1,-0.1)

                    else:
                        self.servo.set_speed(-0.1,0.1)
                    time.sleep(0.005)
                    self.drive(0,0)
                    centered_cam = False

                else:
                    self.drive(0,0)
                    print("done")
            # else:
            #     print(errorObst)


    def stage5(self, speed: float) -> None:
        """
        Obstacle distance + orientation algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        turn_mode = 1
        turntime = 1000

        time.sleep(3)
        while True:
            blobs, img = self.cam.get_blobs()
            img_width = img.width()

            found_obstacle = [blob for blob in blobs if blob[8] == self.obstacle_id + 1]
            print(found_obstacle)

            if len(found_obstacle) != 0:
                largest_obstacle = max(found_obstacle, key=lambda b: b.pixels())
                img.draw_rectangle(largest_obstacle.rect(), color=(0, 255, 0))
                obstacle_x = largest_obstacle.cx()

                if turn_mode == 0:  # turn towards
                    if obstacle_x < img_width // 2:
                        self.drive(speed, 0.5) # Left
                    else:
                        self.drive(speed, -0.5) # Right
                else:  # turn away
                    if obstacle_x < img_width // 2:
                        print('Turning right')
                        self.drive(speed, 0.4) # Right
                    else:
                        print('Turning left')
                        self.drive(speed, -0.4)

                time.sleep_ms(turntime)
                self.drive(0, 0)  # stop
                time.sleep(0.5)

            else:
                print('No obstacle detected')
                sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

                found_mid = self.cam.find_blob(sorted_blobs, self.mid_line_id)
                found_l = self.cam.find_blob(sorted_blobs, self.l_line_id)
                found_r = self.cam.find_blob(sorted_blobs, self.r_line_id)

                self.biasUpdate(sorted_blobs,found_mid,found_l,found_r,None)
                self.drive(speed,self.bias)


            self.servo.soft_reset()
            return

    def stage6(self, speed: float, bias: float) -> None:
        """
        Obstacle distance + orientation algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        turn_mode = 1
        turntime = 1000

        time.sleep(3)
        while True:
            blobs, img = self.cam.get_blobs()
            img_width = img.width()

            found_obstacle = [blob for blob in blobs if blob[8] == self.obstacle_id + 1]
            print(found_obstacle)

            if len(found_obstacle) != 0:
                largest_obstacle = max(found_obstacle, key=lambda b: b.pixels())
                img.draw_rectangle(largest_obstacle.rect(), color=(0, 255, 0))
                obstacle_x = largest_obstacle.cx()

                if turn_mode == 0:  # turn towards
                    if obstacle_x < img_width // 2:
                        self.drive(speed, 0.5) # Left
                    else:
                        self.drive(speed, -0.5) # Right
                else:  # turn away
                    if obstacle_x < img_width // 2:
                        print('Turning right')
                        self.drive(speed, 0.4) # Right
                    else:
                        print('Turning left')
                        self.drive(speed, -0.4)

                time.sleep_ms(turntime)
                self.drive(0, 0)  # stop
                time.sleep(0.5)

            else:
                print('No obstacle detected')
                self.drive(speed, 0)

            time.sleep(0.1)

        self.servo.soft_reset()
        return

    def biasUpdate(self,blobs, mid,left,right,obst,norm1 = 1,norm2 = 2,norm3 = 0.1):
        self.bias = 0
        if mid != None:
            mid_id_blob = blobs[self.cam.find_blob(blobs, self.mid_line_id)]
            errorMid = mid_id_blob.cx() - self.cam.w_centre
            self.bias += self.PID1.get_pid(errorMid,1)/norm1
        if left != None:
            left_id_blob = blobs[self.cam.find_blob(blobs, self.l_line_id)]
            errorLeft = left_id_blob.cx()- sensor.width()
        else:
            errorLeft = 0
        self.bias += self.PID2.get_pid(errorLeft,1)/norm2
        if right != None:
            right_id_blob = blobs[self.cam.find_blob(blobs, self.r_line_id)]
            errorRight = right_id_blob.cx()
        else:
            errorRight = 0
        self.bias += self.PID3.get_pid(errorRight,1)/norm2



    def drive(self, drive: float, steering: float) -> None:
        """
        Differential drive function for the robot.

        Args:
            drive (float): Speed to set the servos to (-1~1)
            steering (float): Sets the steering to (-1~1)
        """
        # Apply limits
        self.servo.set_differential_drive(drive, steering)


    def track_blob(self, blob) -> None:
        """
        Adjust the camera pan angle to track a specified blob based on its ID.

        Args:
            blob: The blob object to be tracked
        """
        # Error between camera angle and target in pixels
        pixel_error = blob.cx() - self.cam.w_centre

        # Convert error to angle
        angle_error = -(pixel_error/sensor.width()*self.cam.h_fov)

        pid_error = self.PID.get_pid(angle_error,1)

        # Error between camera angle and target in ([deg])
        pan_angle = self.servo.pan_pos + pid_error

        # Move pan servo to track block
        self.servo.set_angle(pan_angle)


    def scan_for_blob(self, threshold_idx: int, step = 2, limit = 20) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs_bottom()
            found_idx = self.cam.find_blob(blobs, threshold_idx)
            if found_idx:
                break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                self.scan_direction *= -1


    def debug(self, threshold_idx: int) -> None:
        """
        A debug function for the Robots vision.
        If no block ID is specified, all blocks are detected and debugged.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
        """
        while True:
            blobs, img = self.cam.get_blobs()
            if threshold_idx is not None:
                found_idx = self.cam.find_blob(blobs, threshold_idx)
            else:
                found_idx = range(len(blobs))

            if found_idx:
                for blob in [blobs[i] for i in found_idx]:
                    img.draw_rectangle(blob.rect())
                    img.draw_string(blob.cx(),blob.cy(),str(blob.code()))

                    angle_err = blob.cx() - self.cam.w_centre

                    print('\n' * 2)
                    print('Code:       ', blob.code())
                    print('X-pos:      ',blob.cx())
                    print('Pan angle:  ', self.servo.pan_pos)
                    print('Angle err:  ', angle_err)
                    print('Angle corr: ', (angle_err-self.servo.pan_pos)/self.servo.max_deg)
                    print('Block size: ', blob.pixels())

                    time.sleep(1)


    def reset(self) -> None:
        """
        Resets the servo positions to their default states and waits.
        """
        self.servo.soft_reset()


    def release(self) -> None:
        """
        Release all servos (no wait).
        """
        self.servo.release_all()


if __name__ == "__main__":
    Thresholds =    [(49, 55, 24, 44, -3, 10), #Red
                (10, 20, -18, 5, -14, 11), #Obstacle
                (42, 46, -10, 1, -15, 2), #Purple
                (50, 59, -5, 11, -40, -25), #Blue
                ]
    dude = Robot(Thresholds,10)
    # dude.stage2(0.05)
    # dude.stage3(0.03,0,25)
    dude.stage4(0.05,10,25)
    # dude.stage5(0.05)
    #dude.debug(1)
