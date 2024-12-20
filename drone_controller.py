import cv2
import keyboard as kp
from djitellopy import Tello
import time


class DroneController:
    def __init__(self):
        self.drone = Tello()
        self.drone.connect()
        self.drone.streamon()

    def get_video_frame(self):
        """Отримання кадру відео з дрона та корекція кольору"""
        frame = self.drone.get_frame_read().frame
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frame

    def get_keyboard_input(self):
        """Обробка клавіатурних команд для управління дроном"""
        lr, fb, ud, yv = 0, 0, 0, 0
        speed = 80
        liftSpeed = 80
        moveSpeed = 85
        rotationSpeed = 100

        if kp.is_pressed("LEFT"):
            lr = -speed
        elif kp.is_pressed("RIGHT"):
            lr = speed

        if kp.is_pressed("UP"):
            fb = moveSpeed
        elif kp.is_pressed("DOWN"):
            fb = -moveSpeed

        if kp.is_pressed("w"):
            ud = liftSpeed
        elif kp.is_pressed("s"):
            ud = -liftSpeed

        if kp.is_pressed("d"):
            yv = rotationSpeed
        elif kp.is_pressed("a"):
            yv = -rotationSpeed

        if kp.is_pressed("q"):  # land
            self.drone.land()
            time.sleep(3)
        elif kp.is_pressed("e"):  # take off
            self.drone.takeoff()

        if kp.is_pressed("z"):  # take screenshot
            img = self.get_video_frame()
            filename = f"tellopy/Resources/Images/{time.time()}.jpg"
            cv2.imwrite(filename, img)
            time.sleep(0.3)

        return [lr, fb, ud, yv]

    def send_rc_control(self, commands):
        """Відправка команд на управління дроном"""
        self.drone.send_rc_control(commands[0], commands[1], commands[2], commands[3])

    def take_screenshot(self):
        """Зробити скріншот з відеопотоку"""
        img = self.get_video_frame()
        filename = f"tellopy/Resources/Images/{time.time()}.jpg"
        cv2.imwrite(filename, img)
        time.sleep(0.3)
