# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       LINKS                                                        #
# 	Created:      9/30/2026, 7:25:03 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

brain = Brain()

LEFT = Motor55(Ports.PORT2)

RIGHT = Motor55(Ports.PORT1)

servo_motor = Servo(brain.three_wire_port.a)

servo_motor.set_position(0, DEGREES)

servo_angle = 0

controller_1 = Controller(PRIMARY)

tag_vision = AiVision(Ports.PORT6, AiVision.ALL_TAGS)

closest_tag = {"id": 0, "width": 0, "centerX": 0}

def print_AprilTags_to_brain():
    global closest_tag
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    # Take a snapshot of all AprilTags.

    tags = tag_vision.take_snapshot(AiVision.ALL_TAGS)

    # Print tag data
    if tags and len(tags) > 0:
        for tag in tags:
            brain.screen.print("Found TagID: ")
            brain.screen.print(tag.id)
            brain.screen.print(",    Roll angle: ")
            brain.screen.print(tag.angle)
            brain.screen.next_row()
            brain.screen.print("CenterX: ")
            brain.screen.print(tag.centerX)
            brain.screen.print(",    CenterY: ")
            brain.screen.print(tag.centerY)
            brain.screen.next_row()
            brain.screen.print("Width: ")
            brain.screen.print(tag.width)
            brain.screen.print(",    Height: ")
            brain.screen.print(tag.height)
            brain.screen.next_row()
            brain.screen.next_row()
            if tag.width > closest_tag["width"]:
                closest_tag["id"] = tag.id
                closest_tag["width"] = tag.width
                closest_tag["centerX"] = tag.centerX
    else:
        # If no AprilTags are found in this snapshot, display a message.
        brain.screen.print("No AprilTags")

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")

    error = 160 - closest_tag["centerX"]
    
    # place automonous code here

def user_control():
    global servo_angle
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    # place driver control in this while loop
    
    while True:
        if controller_1.buttonL1.pressing():
            servo_angle += 1
        if controller_1.buttonL2.pressing():
            servo_angle -= 1
        print_AprilTags_to_brain()
        
        LEFT.spin(FORWARD, (controller_1.axis1.position() - controller_1.axis3.position())*12, VOLT)
        RIGHT.spin(FORWARD, (controller_1.axis1.position() + controller_1.axis3.position())*12, VOLT)
        servo_motor.set_position(servo_angle, DEGREES)
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()