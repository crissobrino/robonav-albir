from servos import *
from camera import *
from pid_control import PID
import time

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain = 10,
                p=0.4, i=0.2, d=0.15,
                p2=0.3, i2=0.2, d2=0.15,
                p3=0.3, i3=0, d3=0.9 ,
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
        self.bias = 0

        # Blob IDs
        self.mid_line_id = 0
        self.obstacle_id = 1
        self.l_line_id = 3
        self.r_line_id = 2

        self.scan_direction = 1

    def stage100(self, speed: float):
        while True:
            blobs, img = self.cam.get_blobs()
            sorted_blobs = sorted(blobs, key=lambda b: b.pixels(), reverse=True)

            found_mid = self.cam.find_blob(sorted_blobs, self.mid_line_id)
            found_l = self.cam.find_blob(sorted_blobs, self.l_line_id)
            found_r = self.cam.find_blob(sorted_blobs, self.r_line_id)
            found_obstacle = self.cam.find_blob(sorted_blobs, self.obstacle_id)

            # mid_id_blob = blobs[self.cam.find_blob(sorted_blobs, self.mid_line_id)]
            # left_id_blob = blobs[self.cam.find_blob(sorted_blobs, self.l_line_id)]
            # right_id_blob = blobs[self.cam.find_blob(sorted_blobs, self.r_line_id)]
            # obstacle_blob = blobs[self.cam.find_blob(sorted_blobs, self.obstacle_id)]

            self.biasUpdate(sorted_blobs,found_mid,found_l,found_r,found_obstacle)
            self.drive(speed,self.bias)

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
        # if obst!=None:
        #     print("seeing obstacle")
        #     obstacle_blob = blobs[self.cam.find_blob(blobs, self.obstacle_id)]
        #     if obstacle_blob.cx()<self.cam.w_centre:
        #         errorObst = obstacle_blob.cx() - self.cam.w_centre
        #     else:
        #         errorObst = self.cam.w_centre - obstacle_blob.cx()
        # else:
        #     errorObst = 0
        # self.bias += self.PID4.get_pid(errorObst,1)/norm3


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
    Thresholds =    [(45, 69, 23, 54, 0, 30), #Red
                    (7, 24, -26, -3, -10, 9), #Obstacle
                    (27, 42, 9, 26, -29, -10), #Purple
                    (44, 68, -19, -3, -32, -13),
                    ]
    dude = Robot(Thresholds,10)
    dude.stage100(0.03)
    #dude.debug(1)
