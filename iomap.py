# iomap.py

class Hardware:
    """
    Hardware Abstraction Layer (HAL) for SR robots.
    Provides a unified interface for motors, sensors, bumpers, LEDs, servos, and camera.
    Safe for both simulation and real hardware.
    """
    # in program sr_board.py can now do the following
    #
    # hw = Hardware(robot)
    # sensors = hw.sense()
    #
    # if sensors["bumper_fl"]:
    #    print("Front left bumper pressed!")
    #    print("Front ultrasonic distance:", sensors["ultrasound_front"])
    #    print("Center reflectance:", sensors["reflect_center"])

    # -------------------------------
    # Arduino pin mapping (SR3 standard)
    # -------------------------------
    # Bumpers (digital)
    PIN_BUMPER_FL  = 10
    PIN_BUMPER_FR  = 11
    PIN_BUMPER_RL  = 12
    PIN_BUMPER_RR  = 13

    # Reflectance sensors (analog)
    PIN_REFLECT_LEFT   = "A0"
    PIN_REFLECT_CENTER = "A1"
    PIN_REFLECT_RIGHT  = "A2"

    # Ultrasonic sensors (digital)
    PIN_ULTRASOUND_FRONT_TRIG  = 2
    PIN_ULTRASOUND_FRONT_ECHO  = 3
    PIN_ULTRASOUND_LEFT_TRIG2  = 4
    PIN_ULTRASOUND_LEFT_ECHO2  = 5
    PIN_ULTRASOUND_RIGHT_TRIG  = 6
    PIN_ULTRASOUND_RIGHT_ECHO  = 7
    PIN_ULTRASOUND_BACK_TRIG2  = 8
    PIN_ULTRASOUND_BACK_ECHO2  = 9

    def __init__(self, robot):
        self.robot = robot

        # -------------------------------
        # BUMPERS (digital input)
        # -------------------------------
        self.bumper_front_left  = lambda: robot.arduino.digital_read(self.PIN_BUMPER_FL)
        self.bumper_front_right = lambda: robot.arduino.digital_read(self.PIN_BUMPER_FR)
        self.bumper_rear_left   = lambda: robot.arduino.digital_read(self.PIN_BUMPER_RL)
        self.bumper_rear_right  = lambda: robot.arduino.digital_read(self.PIN_BUMPER_RR)

        # -------------------------------
        # REFLECTANCE SENSORS (analog input)
        # -------------------------------
        self.reflect_left   = lambda: robot.arduino.analog_read(self.PIN_REFLECT_LEFT)
        self.reflect_center = lambda: robot.arduino.analog_read(self.PIN_REFLECT_CENTER)
        self.reflect_right  = lambda: robot.arduino.analog_read(self.PIN_REFLECT_RIGHT)

        # -------------------------------
        # MOTORS
        # -------------------------------
        motors = getattr(robot.motor_board, "motors", [])
        self.motor_left  = motors[0] if len(motors) > 0 else None
        self.motor_right = motors[1] if len(motors) > 1 else None

        # -------------------------------
        # SERVOS
        # -------------------------------
        servos = getattr(robot.servo_board, "servos", [])
        self.servo_arm   = servos[0] if len(servos) > 0 else None
        self.servo_claw  = servos[1] if len(servos) > 1 else None

        # -------------------------------
        # LEDS
        # -------------------------------
        self.led_run   = getattr(robot.power_board, "_run_led", None)
        self.led_error = getattr(robot.power_board, "_error_led", None)
        self.kch_leds  = getattr(getattr(robot, "kch", None), "leds", None)
        self.kch_set_led = getattr(getattr(robot, "kch", None), "set_led", None)

        # -------------------------------
        # PIEZO / BUZZER
        # -------------------------------
        piezo = getattr(robot.power_board, "piezo", None)
        self.piezo      = piezo
        self.piezo_buzz = getattr(piezo, "buzz", None)

        # -------------------------------
        # BATTERY
        # -------------------------------
        battery = getattr(robot.power_board, "battery_sensor", None)
        self.battery_voltage = getattr(battery, "voltage", None)
        self.battery_current = getattr(battery, "current", None)

        # -------------------------------
        # ULTRASONIC SENSORS
        # -------------------------------
        self.ultrasound = lambda trig, echo: getattr(robot.arduino, "ultrasound_measure", lambda t, e: None)(trig, echo)
        self.ultrasound_front  = lambda: self.ultrasound(self.PIN_ULTRASOUND_FRONT_TRIG, self.PIN_ULTRASOUND_FRONT_ECHO)
        self.ultrasound_left2  = lambda: self.ultrasound(self.PIN_ULTRASOUND_LEFT_TRIG2, self.PIN_ULTRASOUND_LEFT_ECHO2)
        self.ultrasound_right  = lambda: self.ultrasound(self.PIN_ULTRASOUND_RIGHT_TRIG, self.PIN_ULTRASOUND_RIGHT_ECHO)
        self.ultrasound_back2  = lambda: self.ultrasound(self.PIN_ULTRASOUND_BACK_TRIG2, self.PIN_ULTRASOUND_BACK_ECHO2)

        # -------------------------------
        # CAMERA
        # -------------------------------
        camera = getattr(robot, "camera", None)
        self.orientation    = getattr(camera, "orientation", None)
        self.camera_see     = getattr(camera, "see", None)
        self.camera_capture = getattr(camera, "capture_image", None)

        # -------------------------------
        # GENERAL BOARD ACCESS
        # -------------------------------
        self.arduino     = getattr(robot, "arduino", None)
        self.motor_board = getattr(robot, "motor_board", None)
        self.servo_board = getattr(robot, "servo_board", None)
        self.power_board = getattr(robot, "power_board", None)
        self.kch         = getattr(robot, "kch", None)

    # -------------------------------
    # UNIFIED SENSOR READ
    # -------------------------------
    def sense(self):
        """
        Returns a dictionary of all sensor readings:
        bumpers, reflectance sensors, and ultrasonics.
        """
        return {
            "bumper_fl": self.bumper_front_left(),
            "bumper_fr": self.bumper_front_right(),
            "bumper_rl": self.bumper_rear_left(),
            "bumper_rr": self.bumper_rear_right(),

            "reflect_left": self.reflect_left(),
            "reflect_center": self.reflect_center(),
            "reflect_right": self.reflect_right(),

            "ultrasound_front": self.ultrasound_front(),
            "ultrasound_left2": self.ultrasound_left2(),
            "ultrasound_right": self.ultrasound_right(),
            "ultrasound_back2": self.ultrasound_back2(),
        }
