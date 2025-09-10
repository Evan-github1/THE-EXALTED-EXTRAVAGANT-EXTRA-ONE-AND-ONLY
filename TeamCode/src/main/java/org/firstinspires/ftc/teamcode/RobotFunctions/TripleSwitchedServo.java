package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.Servo;

/*
TODO: Same thing as DoubleSwitchedServo class but for 3 positions instead of 2
 No clear examples but can be used in special cases (like Team #2's swing)
 */
 
public class TripleSwitchedServo extends DoubleSwitchedServo {

    public int servoSwitch = 0;
    protected double pos3;

    public TripleSwitchedServo(Servo servo1, double pos1, double pos2, double pos3) {
        super(servo1, pos1, pos2);
        this.pos3 = pos3;
    }
    public TripleSwitchedServo(Servo servo1, Servo servo2, double pos1, double pos2, double pos3) {
        super(servo1, servo2, pos1, pos2);
        this.pos3 = pos3;
    }

    // primary and secondary positions are inherited

    public void tertiaryPos() {
        servo1.setPosition(pos3);
        if (servo2 != null) {
            servo2.setPosition(pos3);
        }
    }

    @Override
    public void quickSwitch() {
        if (servoSwitch == 0) {
            primaryPos();
            servoSwitch++;
        } else if (servoSwitch == 1) {
            secondaryPos();
            servoSwitch++;
        } else if (servoSwitch == 2) {
            tertiaryPos();
            servoSwitch = 0;
        }

        Movable.time = System.currentTimeMillis();
    }
}
