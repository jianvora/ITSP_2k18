"""ITSP_2k18
   Team Shikari-ID 2
   Self-aiming gun
   Contibutors-Jian,Rishabh,Dhruv,Siddhant"""


try:
    import cv2
except Exception as e:
    print("\nWARNING:OPENCV NOT INSTALLED.TO TEST MOTION DETECTION MAKE SURE YOU HAVE OPENCV CONFIGURED.")

import time
import thread
import threading
import atexit
import sys
import termios
import contextlib

import imutils
import RPi.GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor


### User Parameters ###

MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False

MAX_STEPS_X = 30
MAX_STEPS_Y = 15

RELAY_PIN = 22

#######################


@contextlib.contextmanager
def raw_mode(file):
    """
    Magic function that allows key presses.
    :param file:
    :return:
    """
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)


class VideoUtils(object):
    """
    Helper functions for video utilities.
    """
    @staticmethod
    def live_video(camera_port=0):
       

        video_capture = cv2.VideoCapture(camera_port)

        while True:
            # Capture frame-by-frame
            ret, frame = video_capture.read()

            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, camera_port=0, show_video=False):

        camera = cv2.VideoCapture(camera_port)
        time.sleep(0.25)

        # initialize the first frame in the video stream
        firstFrame = None
        tempFrame = None
        count = 0

        # loop over the frames of the video
        while True:
        

            (grabbed, frame) = camera.read()

        
            if not grabbed:
                break

            # resize the frame, convert it to grayscale, and blur it
            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            # if the first frame is None, initialize it
            if firstFrame is None:
                print "WAITING FOR VIDEO TO ADJUST..."
                if tempFrame is None:
                    tempFrame = gray
                    continue
                else:
                    delta = cv2.absdiff(tempFrame, gray)
                    tempFrame = gray
                    tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
                    tst = cv2.dilate(tst, None, iterations=2)
                    if count > 30:
                        print "DONE.WAITING FOR MOTION."
                        if not cv2.countNonZero(tst) > 0:
                            firstFrame = gray
                        else:
                            continue
                    else:
                        count += 1
                        continue

            # compute the absolute difference between the current frame and
            # first frame
            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            thresh = cv2.dilate(thresh, None, iterations=2)
            c = VideoUtils.get_best_contour(thresh.copy(), 5000)

            if c is not None:
                # compute the bounding box for the contour, draw it on the frame,
                # and update the text
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(c, frame)

            if show_video:
                cv2.imshow("SECURITY FEED", frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):
                    break

        camera.release()
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        im, contours, hierarchy = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt
   class Turret(object):
    """
    Class used for turret control.
    """
    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode

        # create a default object, no changes to I2C address or frequency
        self.mh = Adafruit_MotorHAT()
        atexit.register(self.__turn_off_motors)

        # Stepper motor 1
        self.sm_x = self.mh.getStepper(200, 1)      # 200 steps/rev, motor port #1
        self.sm_x.setSpeed(5)                       # 5 RPM
        self.current_x_steps = 0

        # Stepper motor 2
        self.sm_y = self.mh.getStepper(200, 2)      # 200 steps/rev, motor port #2
        self.sm_y.setSpeed(5)                       # 5 RPM
        self.current_y_steps = 0

        # Relay
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_PIN, GPIO.OUT)
        GPIO.output(RELAY_PIN, GPIO.LOW)

    def calibrate(self):
        
        print "PLEASE CALIBRATE THE TILT OF THE GUN SO THAT IT IS LEVEL.\n COMMANDS:\n (w) MOVES UP\n (s) MOVES DOWN\n PRESS (ENTER) TO FINISH.\n"
              
        self.__calibrate_y_axis()

        print "PLEASE CALIBRATE THE YAW OF THE GUN TO ALIGN WITH THE CAMERA.\n COMMANDS:\n (a) MOVES LEFT\n (d) MOVES RIGHT\nPRESS (ENTER) TO FINISH" \
              
        self.__calibrate_x_axis()

        print "CALIBRATION FINISHED."

    def __calibrate_x_axis(self):
        
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    elif ch == "a":
                        if MOTOR_X_REVERSED:
                            Turret.move_backward(self.sm_x, 5)
                        else:
                            Turret.move_forward(self.sm_x, 5)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            Turret.move_forward(self.sm_x, 5)
                        else:
                            Turret.move_backward(self.sm_x, 5)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print "ERROR:UNABLE TO CALIBRATE TURRET.EXITING..."
                sys.exit(1)

    def __calibrate_y_axis(self):
        
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    if ch == "w":
                        if MOTOR_Y_REVERSED:
                            Turret.move_forward(self.sm_y, 5)
                        else:
                            Turret.move_backward(self.sm_y, 5)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            Turret.move_backward(self.sm_y, 5)
                        else:
                            Turret.move_forward(self.sm_y, 5)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print "ERROR:UNABLE TO CALIBRATE TURRET.EXITING..."
                sys.exit(1)

    def motion_detection(self, show_video=False):
     
        VideoUtils.find_motion(self.__move_axis, show_video=show_video)

    def __move_axis(self, contour, frame):
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)

        # find height
        target_steps_x = (2*MAX_STEPS_X * (x + w / 2) / v_w) - MAX_STEPS_X
        target_steps_y = (2*MAX_STEPS_Y*(y+h/2) / v_h) - MAX_STEPS_Y

        print "x: %s, y: %s" % (str(target_steps_x), str(target_steps_y))
        print "current x: %s, current y: %s" % (str(self.current_x_steps), str(self.current_y_steps))

        t_x = threading.Thread()
        t_y = threading.Thread()
        t_fire = threading.Thread()

        # move x
        if (target_steps_x - self.current_x_steps) > 0:
            self.current_x_steps += 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=Turret.move_forward, args=(self.sm_x, 2,))
            else:
                t_x = threading.Thread(target=Turret.move_backward, args=(self.sm_x, 2,))
        elif (target_steps_x - self.current_x_steps) < 0:
            self.current_x_steps -= 1
            if MOTOR_X_REVERSED:
                t_x = threading.Thread(target=Turret.move_backward, args=(self.sm_x, 2,))
            else:
                t_x = threading.Thread(target=Turret.move_forward, args=(self.sm_x, 2,))

        # move y
        if (target_steps_y - self.current_y_steps) > 0:
            self.current_y_steps += 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=Turret.move_backward, args=(self.sm_y, 2,))
            else:
                t_y = threading.Thread(target=Turret.move_forward, args=(self.sm_y, 2,))
        elif (target_steps_y - self.current_y_steps) < 0:
            self.current_y_steps -= 1
            if MOTOR_Y_REVERSED:
                t_y = threading.Thread(target=Turret.move_forward, args=(self.sm_y, 2,))
            else:
                t_y = threading.Thread(target=Turret.move_backward, args=(self.sm_y, 2,))

        # fire if necessary
        if not self.friendly_mode:
            if abs(target_steps_y - self.current_y_steps) <= 2 and abs(target_steps_x - self.current_x_steps) <= 2:
                t_fire = threading.Thread(target=Turret.fire)

        t_x.start()
        t_y.start()
        t_fire.start()

        t_x.join()
        t_y.join()
        t_fire.join()

    def interactive(self):
        

        Turret.move_forward(self.sm_x, 1)
        Turret.move_forward(self.sm_y, 1)

        print 'COMMANDS:\n(w) UP\n(s) DOWN\n (a) LEFT\n (d) RIGHT\n'
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch or ch == "q":
                        break

                    if ch == "w":
                        if MOTOR_Y_REVERSED:
                            Turret.move_forward(self.sm_y, 5)
                        else:
                            Turret.move_backward(self.sm_y, 5)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            Turret.move_backward(self.sm_y, 5)
                        else:
                            Turret.move_forward(self.sm_y, 5)
                    elif ch == "a":
                        if MOTOR_X_REVERSED:
                            Turret.move_backward(self.sm_x, 5)
                        else:
                            Turret.move_forward(self.sm_x, 5)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            Turret.move_forward(self.sm_x, 5)
                        else:
                            Turret.move_backward(self.sm_x, 5)
                    elif ch == "\n":
                        Turret.fire()

            except (KeyboardInterrupt, EOFError):
                pass

    @staticmethod
    def fire():
        GPIO.output(RELAY_PIN, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(RELAY_PIN, GPIO.LOW)

    @staticmethod
    def move_forward(motor, steps):
        """
        Moves the stepper motor forward the specified number of steps.
        :param motor:
        :param steps:
        :return:
        """
        motor.step(steps, Adafruit_MotorHAT.FORWARD,  Adafruit_MotorHAT.INTERLEAVE)

    @staticmethod
    def move_backward(motor, steps):
        """
        Moves the stepper motor backward the specified number of steps
        :param motor:
        :param steps:
        :return:
        """
        motor.step(steps, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.INTERLEAVE)

    def __turn_off_motors(self):
        """
        Recommended for auto-disabling motors on shutdown!
        :return:
        """
        self.mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
        self.mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

if __name__ == "__main__":
    t = Turret(friendly_mode=False)

    user_input = raw_input("\nCHOOSE AN INPUT MODE: \n(1) MOTION DETECTION \n(2) INTERACTIVE\n")

    if user_input == "1":
        t.calibrate()
        if raw_input("LIVE VIDEO? (y, n)\n").lower() == "y":
            t.motion_detection(show_video=True)
        else:
            t.motion_detection()
    elif user_input == "2":
        if raw_input("LIVE VIDEO? (y, n)\n").lower() == "y":
            thread.start_new_thread(VideoUtils.live_video, ())
        t.interactive()
    else:
        print "\nUNKNOWN INPUT MODE.PLEASE CHOOSE 1 OR 2."

