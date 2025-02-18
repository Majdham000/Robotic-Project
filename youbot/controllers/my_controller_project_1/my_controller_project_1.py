"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import time

KUKA_WHEEL_RADUIS = 0.05 
KUKA_RADUIS = 0.27 
KUKA_MAX_VELCODITY = 14.8 

# PID Factors
Kp = 0.01
Kd = 0.01
Ki = 0

# Last error to be used by the PID.
last_error = 0

#Integral (the accumulation of errors) to be used by the PID.
integral = 0

colors = {}
i = 0
R = False
G = False
B = False
Y = False
S = False

is_gripped = False

#red 1 is original
red_box_1 = [-0.005545071937329631, 2.9530085265814922, 0.10189653264638804]
red_box_2 = [-0.005545071937329631, 2.9130085265814922, 0.10189653264638804]
red_position = [-2.9056364140030717, 2.908440556676168, 0.10191558560384266]
#red_box_2 = [-0.00012873642488892445, -3.646485802830518, 0.10193771737102325]
yellow_box = [-0.0019940175817976164, 0.712910235420672, 0.10189635656542237]
yellow_position = [-2.922187282939701, 0.7184232294128279, 0.10193530978716142]
#yellow_box_2 = [5.234127829930278e-05, -2.996522629452369, 0.10193771737075434]
blue_box = [0.006244583883067277, 1.802960775854616, 0.10188356525682823]
blue_position = [-2.923894451952144, 1.7884673463162166, 0.1019110791766989]
#blue_box_2 = [0.0035641475338084746, -2.4567790686479762, 0.10133405123468876]
green_box_1 = [0.006244583883067277, 1.902960775854616, 0.10188356525682823]
green_box_2 = [0.006244583883067277, 1.802960775854616, 0.10188356525682823]
green_position = [2.8264115928197766, 1.797739823383951, 0.10193408685043165]
#end_point = [0.0008883153851472163, -0.8067713474137689, 0.1012388532495023]

#blue_box = [2.852663781157873e-05, -2.466442369892385, 0.10193771735674603]
#yellow_box = [5.234127829930278e-05, -2.996522629452369, 0.10193771737075434]

initial_position = [0, 0, 0, 0, 0]
desired_position = [0, -1.04, -1.4, -0.85, 0]
wall_position = [0, -0.2, -1.47, -1.35, 0]
desired_velocity = [0.5, 0.5, 0.5, 0.5, 0.5] 
dish_position = [0, 0.525, 1.1, 1.4, 0]
#dish_position = [0, 0.7, 0.9, 1.1, 0]

# Box sensor = -0.09 (z)



def range_conversion(s_start, s_end, d_start, d_end, value):
    """
    This function is responsible for mapping ranges
    examples:
    the mapping of the value 50 from range 0 -> 200 to range -50 -> 50 will be -25
    """
    ration = abs((value - s_start) / (s_end - s_start))
    if(d_start < d_end):
        return  d_start + abs(d_end - d_start) * ration 
    if(d_start > d_end):
        return  d_start - abs(d_end - d_start) * ration 
    

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)
        self.timestep = int(self.getBasicTimeStep())

        self.front_right_wheel = self.getDevice("wheel1")
        self.front_left_wheel = self.getDevice("wheel2")
        self.back_right_wheel = self.getDevice("wheel3")
        self.back_left_wheel = self.getDevice("wheel4")

        self.arm1 = self.getDevice("arm1")
        self.arm2 = self.getDevice("arm2")
        self.arm3 = self.getDevice("arm3")
        self.arm4 = self.getDevice("arm4")
        self.arm5 = self.getDevice("arm5")
        self.left_finger = self.getDevice("finger::left")
        self.right_finger = self.getDevice("finger::right")
        
        self.sensor = self.getDevice("wheel1sensor")
        self.sensor.enable(self.timestep)
        
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timestep)

        # self.gyro = self.getDevice('gyro') 
        # self.gyro.enable(self.timestep)
        
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)
        
        self.box_sensor = self.getDevice("box_sensor")
        self.box_sensor.enable(self.timestep)

        self.box_sensor_1 = self.getDevice("box_sensor(1)")
        self.box_sensor_1.enable(self.timestep)

        # self.compass = self.getDevice('compass')
        # self.compass.enable(self.timestep)

        self.receiver = self.getDevice("receiver") 
        self.receiver.enable(self.timestep)

        self.emitter = self.getDevice("emitter")

        self.arm1_sensor = self.arm1.getPositionSensor()
        self.arm2_sensor = self.arm2.getPositionSensor() 
        self.arm3_sensor = self.arm3.getPositionSensor() 
        self.arm4_sensor = self.arm4.getPositionSensor() 
        self.arm5_sensor = self.arm5.getPositionSensor()
        
        self.arm1_sensor.enable(self.timestep) 
        self.arm2_sensor.enable(self.timestep) 
        self.arm3_sensor.enable(self.timestep) 
        self.arm4_sensor.enable(self.timestep) 
        self.arm5_sensor.enable(self.timestep)


        #self.step()

        #self.arm1.setPosition(float("inf"))
        #self.arm2.setPosition(float("inf"))
        #self.arm3.setPosition(float("inf"))
        #self.arm4.setPosition(float("inf"))
        #self.arm5.setPosition(float("inf"))
        #self.left_finger.setPosition(float("inf"))
        #self.right_finger.setPosition(float("inf"))

        #self.arm1.setVelocity(-0.1)
        #self.arm2.setVelocity(0)
        #self.arm3.setVelocity(0)
        #self.arm4.setVelocity(0)
        #self.arm5.setVelocity(0)
        #self.left_finger.setVelocity(0)
        #self.right_finger.setVelocity(0)

        self.front_right_wheel.setPosition(float("inf"))
        self.front_left_wheel.setPosition(float("inf"))
        self.back_right_wheel.setPosition(float("inf"))
        self.back_left_wheel.setPosition(float("inf"))

        self.front_right_wheel.setVelocity(0)
        self.front_left_wheel.setVelocity(0)
        self.back_right_wheel.setVelocity(0)
        self.back_left_wheel.setVelocity(0)


        self.sensors = list(map(lambda v: self.getDevice(f"lfs{v}"), range(8)))

        self.weights = [-1000, -1000, -1000, -1000, 1000, 1000, 1000, 1000]

        for sensor in self.sensors:
            #print("enabled: ", sensor)
            sensor.enable(self.timestep)

        self.step(self.timestep)

    def get_sensors_value(self):
        value = 0

        for index, sensor in enumerate(self.sensors):
            if(sensor.getValue() == 1000):
                #print(sensor.getValue())
                value += self.weights[index]

        return value
    
    def run_motors_stearing(self, stearing, velocity = KUKA_MAX_VELCODITY):
        """
        A function that is responsible for the steering functionality for the motor
        Steering value:
            - from -100 to 0 will turn left
            - from 0 to 100 will turn right
            - if equals 100 will turn the robot around it self to the right
            - if equals -100 will turn the robot around it self to the left
            - if equals 50 will turn the robot around right wheel to the right
            - if equals -50 will turn the robot around left wheel to the left
        """
        front_right_velocity = velocity if stearing < 0 else range_conversion(0, 100, velocity, -velocity, stearing)
        front_left_velocity = velocity if stearing > 0 else range_conversion(0, -100, velocity, -velocity, stearing)
        back_right_velocity = velocity if stearing < 0 else range_conversion(0, 100, velocity, -velocity, stearing)
        back_left_velocity = velocity if stearing > 0 else range_conversion(0, -100, velocity, -velocity, stearing)
        self.set_motors_velocity(front_right_velocity, front_left_velocity, back_right_velocity, back_left_velocity)

    def PID_step(self, velocity = KUKA_MAX_VELCODITY):
        global last_error, integral
        value = self.get_sensors_value()
        #print(value)
        error = 0 - value
        # Get P term of the PID.
        P = Kp * error
        # Get D term of the PID.
        D = Kd * (last_error - error)
        # Update last_error to be used in the next iteration.
        last_error = error
        # Get I term of the PID.
        I = Ki * integral
        # Update intergral to be used in the next iteration.
        integral += error

        steering = P + D + I
        self.run_motors_stearing(steering,velocity)
        
        
    
    def close_gripper(self):
        self.left_finger.setVelocity(0.01)
        self.right_finger.setVelocity(0.01)

        self.left_finger.setPosition(0)  
        self.right_finger.setPosition(0)

    
    
    def open_gripper(self):
        self.left_finger.setVelocity(0.01)
        self.right_finger.setVelocity(0.01)

        self.left_finger.setPosition(0.025)  
        self.right_finger.setPosition(0.025)





    def set_motors_velocity(self, wheel1_v, wheel2_v, wheel3_v, wheel4_v):
        self.front_right_wheel.setVelocity(wheel1_v)
        self.front_left_wheel.setVelocity(wheel2_v)
        self.back_right_wheel.setVelocity(wheel3_v)
        self.back_left_wheel.setVelocity(wheel4_v)

    def move_forward(self, velocity):
        self.set_motors_velocity(velocity, velocity, velocity, velocity)

    def move_backward(self, velocity):
        self.set_motors_velocity(-velocity, -velocity, -velocity, -velocity)

    def move_left(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def move_right(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)

    def turn_cw(self, velocity):
        self.front_right_wheel.setVelocity(-velocity)
        self.front_left_wheel.setVelocity(velocity)
        self.back_left_wheel.setVelocity(velocity)
        self.back_right_wheel.setVelocity(-velocity)

    def turn_ccw(self, velocity):
        self.front_right_wheel.setVelocity(velocity)
        self.front_left_wheel.setVelocity(-velocity)
        self.back_left_wheel.setVelocity(-velocity)
        self.back_right_wheel.setVelocity(velocity)
        
    
        
        
    def colors_order(self):
        # print(dir(self.camera))
        global colors
        cameraArray = self.camera.getImageArray()
        red = cameraArray[0][0][0]
        green = cameraArray[0][0][1]
        blue = cameraArray[0][0][2]
        
        global R
        global G
        global B
        global Y
        global i

        if R == False:
            if green == 0 and blue == 0:
                colors[i] = "red"
                i += 1
                R = True
                
        if G == False:
            if red == 0 and blue == 0:
                colors[i] = "green"
                i += 1
                G = True
                
        if B == False:
            if green == 0 and red == 0: 
                colors[i] = "blue"
                i += 1
                B = True
                
        if Y == False:
            if green != 0 and red != 0 and blue == 0 : 
                colors[i] = "yellow"
                i += 1
                Y = True

        # print(red,"\t", green ,"\t",blue)
        # print("---------------")
        
        
    def run_motors_for_rotations_by_right_motor(
        self,
        rotations,
        wheel1_v = KUKA_MAX_VELCODITY,
        wheel2_v = KUKA_MAX_VELCODITY,
        wheel3_v = KUKA_MAX_VELCODITY,
        wheel4_v = KUKA_MAX_VELCODITY
        ):
        """
            A funtion that will run two motors for specafic velocity
            for specafic rotations
            and will depend on right motor sensor to calculate rotations
        """
    
        # calculate angle = number_of_rotations / 2 * PI
        angle = rotations * 2 * math.pi
        
        # get first value of the sensor
        curr = self.sensor.getValue()
    
        # set motors velocities
        self.set_motors_velocity(wheel1_v, wheel2_v, wheel3_v, wheel4_v)
    
        # do stepping until the differance between initial sensor value
        # and current value is less than the required angle
        while(abs(self.sensor.getValue() - curr) < angle):
            #print(self.sensor.getValue() - curr)
            #print(angle)
            self.step(self.timestep)
    
        # reset motors velocities to zero
        self.set_motors_velocity(0, 0, 0, 0)
        

    def move_distance_by_right_motor(
        self,
        distance,
        wheel1_v = KUKA_MAX_VELCODITY,
        wheel2_v = KUKA_MAX_VELCODITY,
        wheel3_v = KUKA_MAX_VELCODITY,
        wheel4_v = KUKA_MAX_VELCODITY
    ):
        """
            A funtion that will move the robot by specafic distance,
            with specafic motors velocities,
            and will depend on right motor sensor to calculate the distance
        """

        rotations = distance / (2 * math.pi * KUKA_WHEEL_RADUIS)
        self.run_motors_for_rotations_by_right_motor(
            rotations,
            wheel1_v,
            wheel2_v,
            wheel3_v,
            wheel4_v
        )

    
    def turn_angle(self, angle):
        """
            A funtion that will turn the robot by specafic angle (in degrees) counterclockwise
        """
        distance = (2 * math.pi * KUKA_RADUIS) / (360 / angle)
        self.move_distance_by_right_motor(
            distance,
            -14.8,
            14.8,
            -14.8,
            14.8
        ) 

    def turn_angle_c(self, angle):
        """
            A funtion that will turn the robot by specafic angle (in degrees) counterclockwise
        """
        distance = (2 * math.pi * KUKA_RADUIS) / (360 / angle)
        self.move_distance_by_right_motor(
            distance,
            14.8,
            -14.8,
            14.8,
            -14.8
        ) 


    def loop(self):
        #global S
        #global red_box, yellow_box, green_box, blue_box
        #global start_time
        global is_gripped
        b1 = True
        b2 = True

        present_color_1 = {}
        present_color_2 = {}
        present_position = {}

        # in order
        #self.step(self.timestep * 1650) 
        while self.step(self.timestep) != -1:
            if self.receiver.getQueueLength() > 0:
                message = self.receiver.getString()
                #print("Received message from Giver: ", message)
                if message == "Start":
                    self.receiver.nextPacket()
                    break

        while self.step(self.timestep) != -1:
            if len(colors) == 4: 
                print("TAKER_COLORS: ", colors)
                break
            else:
                self.colors_order()
                
            self.move_forward(14.8)

        i = True
        while self.step(self.timestep) != -1:

            self.PID_step(10)

            if self.box_sensor.getValue() < 1000 or self.box_sensor_1.getValue() < 1000:
                self.set_motors_velocity(0, 0, 0, 0)
                #print("Box_sensor: ", self.box_sensor.getValue())
                self.open_gripper()
                self.move_arm_to_position(wall_position, desired_velocity)
                self.close_gripper() 
                self.step(self.timestep * 150)

                #BONUS
                if b1 == True:
                    self.move_arm_to_position(dish_position, desired_velocity)
                    self.open_gripper()

                    message = "Bonus"
                    message = message.encode('utf-8')  
                    self.emitter.send(message)

                    while self.step(self.timestep) != -1:
                        if self.receiver.getQueueLength() > 0:
                            message = self.receiver.getString()
                            #print("Received message from Giver: ", message)
                            if message == "Bonus":
                                self.receiver.nextPacket()
                                break

                    self.move_arm_to_position(initial_position, desired_velocity)
                    self.move_arm_to_position(wall_position, desired_velocity)
                    self.close_gripper() 
                    self.step(self.timestep * 150)

                    b1 = False

                ##########

                self.move_arm_to_position(initial_position, desired_velocity)
                self.step(self.timestep * 50)

                self.turn_angle(235)

                message = "Move"
                message = message.encode('utf-8')  
                self.emitter.send(message)
                #print("sent message to Taker: ", message)

                break

        
        for color in colors:
            #print(colors[color])
            if colors[color] == "red": 
                present_color_1 = red_box_1
                present_color_2 = red_box_2
                present_position = red_position
            elif colors[color] == "yellow": 
                present_color_1 = yellow_box
                present_color_2 = yellow_box
                present_position = yellow_position
            elif colors[color] == "blue": 
                present_color_1 = blue_box
                present_color_2 = blue_box
                present_position = blue_position
            elif colors[color] == "green":
                present_color_1 = green_box_1
                present_color_2 = green_box_2
                present_position = green_position

            if colors[0] == colors[color]: rounds = 3
            else: rounds = 4


            for i in range(rounds):
                #print("Taker: ")
                #print(i+1, colors[color], " box")

                while self.step(self.timestep) != -1:
                    # compass_values = self.compass.getValues() 
                    # direction = math.atan2(compass_values[0], compass_values[2])
                    # direction_degrees = math.degrees(direction) 
                    #print(f"Robot direction: {direction_degrees} degrees")


                    i = True
                    while self.step(self.timestep) != -1:
                        self.PID_step(10)

                        if colors[color] == "green":
                            if list(map(lambda x: round(x, 1), self.gps.getValues())) == list(map(lambda x: round(x, 1), present_color_1)) and i == True:
                                #self.move_distance_by_right_motor(0.025)
                                self.turn_angle(126)
                                #print("TURN")
                                i = False

                        else:
                            if list(map(lambda x: round(x, 1), self.gps.getValues())) == list(map(lambda x: round(x, 1), present_color_1)) and i == True:
                                #self.move_distance_by_right_motor(0.025)
                                self.turn_angle_c(126)
                                #print("TURN")
                                i = False

                                
                        #4
                        if list(map(lambda x: round(x, 1), self.gps.getValues())) == list(map(lambda x: round(x, 1), present_position)):
                            self.set_motors_velocity(0, 0, 0, 0)
                            self.move_arm_to_position(desired_position, desired_velocity)
                            self.open_gripper()
                            self.step(self.timestep * 150)
                            self.move_arm_to_position(initial_position, desired_velocity)
                            self.close_gripper()
                            self.step(self.timestep * 50)

                            # BONUS
                            if b2 == True:
                                self.open_gripper()
                                self.move_arm_to_position(dish_position, desired_velocity)
                                self.close_gripper()
                                self.step(self.timestep * 150)

                                self.move_arm_to_position(desired_position, desired_velocity)
                                self.open_gripper()
                                self.step(self.timestep * 150)

                                self.move_arm_to_position(initial_position, desired_velocity)
                                b2 = False
                            ############

                            self.turn_angle(235)

                            # in order
                            #self.step(self.timestep * 1000)
                            while self.step(self.timestep) != -1:
                                if self.receiver.getQueueLength() > 0:
                                    message = self.receiver.getString()
                                    #print("Received message from Giver: ", message)
                                    if message == "Move":
                                        self.receiver.nextPacket()  
                                        break
                            break
                    
                    i = True
                    while self.step(self.timestep) != -1:

                        self.PID_step(10)

                        if colors[color] == "green":
                            if list(map(lambda x: round(x, 1), self.gps.getValues())) == list(map(lambda x: round(x, 1), present_color_2)) and i == True:
                                self.move_distance_by_right_motor(0.030)
                                self.turn_angle_c(126)
                                i = False

                        else:
                            if list(map(lambda x: round(x, 1), self.gps.getValues())) == list(map(lambda x: round(x, 1), present_color_2)) and i == True:
                                self.move_distance_by_right_motor(0.030)
                                self.turn_angle(126)
                                i = False

                        if self.box_sensor.getValue() < 1000 or self.box_sensor_1.getValue() < 1000:
                            self.set_motors_velocity(0, 0, 0, 0)
                            #print("Box_sensor: ", self.box_sensor.getValue())
                            self.open_gripper()
                            self.move_arm_to_position(wall_position, desired_velocity)
                            self.close_gripper() 
                            self.step(self.timestep * 150)

                            self.move_arm_to_position(initial_position, desired_velocity)
                            self.step(self.timestep * 50)

                            self.turn_angle(235)

                            # in order
                            message = "Move"
                            message = message.encode('utf-8')  
                            self.emitter.send(message)
                            #print("sent message to Giver: ", message)

                            break
                    break


    def move_arm_to_position(self, position, velocity):
        #self.open_gripper()
        # Set target positions and velocities
        self.arm1.setPosition(position[0])
        self.arm2.setPosition(position[1])
        self.arm3.setPosition(position[2])
        self.arm4.setPosition(position[3])
        self.arm5.setPosition(position[4])

        self.arm1.setVelocity(velocity[0])
        self.arm2.setVelocity(velocity[1])
        self.arm3.setVelocity(velocity[2])
        self.arm4.setVelocity(velocity[3])
        self.arm5.setVelocity(velocity[4])

        # Wait until the arm reaches the target position
        while not self.is_position_reached(position):
            self.step(self.timestep)

        # Stop the motors
        self.arm1.setVelocity(0)
        self.arm2.setVelocity(0)
        self.arm3.setVelocity(0)
        self.arm4.setVelocity(0)
        self.arm5.setVelocity(0)

        #print("END")
        
        #self.close_gripper()  


    def is_position_reached(self, position):
        # Check if the current positions are close to the target positions
        current_positions = [
            self.arm1_sensor.getValue(),
            self.arm2_sensor.getValue(),
            self.arm3_sensor.getValue(),
            self.arm4_sensor.getValue(),
            self.arm5_sensor.getValue()
        ]
        return all(abs(current - target) < 0.01 for current, target in zip(current_positions, position))
        

#1, 5 FORWARD
#2, 3, 4 BACKWARD

r = RobotController()
#r.step(r.timestep * 1650)
r.loop()
#print(r.gps.getValues())


#lfs7 0.06 => Y
#lfs3 - 0.06 => Y

'''
for color in colors:
    if color == "red": 
        present_color_1 = red_box
        present_color_2 = red_box
    elif color == "yellow": 
        present_color_1 = yellow_box
        present_color_2 = yellow_box
    elif color == "blue": 
        present_color_1 = blue_box_1
        present_color_2 = blue_box_2
    elif color == "green":
        present_color_1 = green_box
        present_color_2 = green_box
'''