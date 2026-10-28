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

tag_vision = AiVision(Ports.PORT20, AiVision.ALL_TAGS)

closest_tag = {"id": 0, "width": 0, "centerX": 0, "centerY": 0}

driveLoopsNoTags = 0

def get_apriltags():
    global closest_tag
    global driveLoopsNoTags
    brain.screen.clear_screen()
    brain.screen.set_cursor(1, 1)
    # Take a snapshot of all AprilTags.

    tags = tag_vision.take_snapshot(AiVision.ALL_TAGS)

    # Print tag data
    if tags and len(tags) > 0:
        driveLoopsNoTags = 0
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
                closest_tag["centerY"] = tag.centerY
        return True
    else:
        # If no AprilTags are found in this snapshot, display a message.
        brain.screen.print("No AprilTags")
        return False

def drive(speed, turn):
    LEFT.spin(FORWARD, (speed + turn)*12, VOLT)
    RIGHT.spin(FORWARD, (speed - turn)*12, VOLT)

"""def auton_return(corrections):
    drive(-0.2, 0)
    wait(1000, MSEC)
    #I fixed wait(1000, MSEC)
    for correction in corrections:
        drive(0, -correction)
        wait(20, MSEC)
    drive(0.5, 0)"""

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    driveLoopsNoTags = 0

    amount_turned = 0

    corrections = []

    while True:
        if get_apriltags():
            driveLoopsNoTags = 0
        else:
            driveLoopsNoTags += 1
        error = 120 - closest_tag["centerY"]
        brain.screen.print("CenterX:", closest_tag["centerX"])
        brain.screen.print("    CenterY:", closest_tag["centerY"])
        brain.screen.print("    error:", error)
        if driveLoopsNoTags < 100:
            if abs(error) > 10:
                drive(0, -error/120)
                corrections.append(-error/120)
            else:
                drive(0.1, -error/120)
                corrections.append(-error/120)
        else:
            drive(0, 0)
        wait(20, MSEC)

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
        #print_AprilTags_to_brain()
        
        drive(controller_1.axis3.position(), controller_1.axis1.position())
        servo_motor.set_position(servo_angle, DEGREES)
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()