package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.Servo;

/*
TODO: this class is useful for servos that utilize two positions
 For example, a claw (open and closed position)
 */

public class DoubleSwitchedServo {

    public boolean servoSwitch = false; // secondary pos
    protected Servo servo1, servo2;
    protected double pos1, pos2;

    public DoubleSwitchedServo(Servo servo1, double pos1, double pos2) {
        this.servo1 = servo1;
        this.pos1 = pos1;
        this.pos2 = pos2;
        setDirections();
    }
    public DoubleSwitchedServo(Servo servo1, Servo servo2, double pos1, double pos2) {
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.pos1 = pos1;
        this.pos2 = pos2;
        setDirections();
    }

    public void primaryPos() {
        servo1.setPosition(pos1);
        if (servo2 != null) {
            servo2.setPosition(pos1);
        }
    }

    public void secondaryPos() {
        servo1.setPosition(pos2);
        if (servo2 != null) {
            servo2.setPosition(pos2);
        }
    }

    public void quickSwitch() {
        if (!servoSwitch) {
            primaryPos();
        } else {
            secondaryPos();
        }
        servoSwitch = !servoSwitch;
        Movable.time = System.currentTimeMillis();
    }

    // use anonymous classes to change, these are default
    public void setDirections() {
        servo1.setDirection(Servo.Direction.FORWARD);
        if (servo2 != null) {
            servo2.setDirection(Servo.Direction.REVERSE);
        }
    }
}
