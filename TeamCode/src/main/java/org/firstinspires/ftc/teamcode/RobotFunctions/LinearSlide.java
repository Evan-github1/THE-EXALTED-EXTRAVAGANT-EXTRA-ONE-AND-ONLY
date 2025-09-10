package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// TODO: exactly what the class name sounds like, might create a class just for general motors

public class LinearSlide {
    private DcMotor LMotor, RMotor;
    private double speed;

    public LinearSlide (DcMotor LMotor, DcMotor RMotor, double speed) {
        this.LMotor = LMotor;
        this.RMotor = RMotor;
        this.speed = speed;
    }

    public void ascend() {
        LMotor.setPower(speed);
        RMotor.setPower(speed);
    }

    public void descend() {
        LMotor.setPower(-speed);
        RMotor.setPower(-speed);
    }

    public void disablePower() {
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public void changeSpeed(double newSpeed) {
        speed = newSpeed;
    }
}
