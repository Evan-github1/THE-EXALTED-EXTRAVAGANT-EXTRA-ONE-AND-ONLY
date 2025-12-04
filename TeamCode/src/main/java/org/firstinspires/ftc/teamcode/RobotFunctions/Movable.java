package org.firstinspires.ftc.teamcode.RobotFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// TODO: inherit this class to be able to drive
public abstract class Movable extends LinearOpMode {
    static protected DcMotor FLW;
    static protected DcMotor BLW;
    static protected DcMotor FRW;
    static protected DcMotor BRW;
    static protected long time;

    static protected double angle, desVol, vx, vy, v1, v2, max;

    @Override
    public void runOpMode() throws InterruptedException {
        time = System.currentTimeMillis();
        FLW = hardwareMap.get(DcMotor.class, "FLW");
        BLW = hardwareMap.get(DcMotor.class, "BLW");
        FRW = hardwareMap.get(DcMotor.class, "FRW");
        BRW = hardwareMap.get(DcMotor.class, "BRW");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }



    // all robots have wheels so this method is a must
    protected void moveWheels(float x, float y) {
        double correctedX = -x;
        angle = Math.atan2(y, correctedX);
        desVol = Math.sqrt(Math.pow(correctedX, 2) + Math.pow(y, 2));

        vx = desVol * Math.cos(angle);
        vy = desVol * Math.sin(angle);
        v1 = vy + vx;
        v2 = vy - vx;
        max = Math.max(Math.abs(v1), Math.abs(v2));

        if(max > 1){
            v1 /= max;
            v2 /= max;
        }

        FLW.setPower(-v1);
        FRW.setPower(-v2);
        BRW.setPower(-v1);
        BLW.setPower(-v2);
    }

    protected void strafe() {
        if (gamepad1.left_bumper) {
            FLW.setPower(-1);
            FRW.setPower(1);
            BLW.setPower(-1);
            BRW.setPower(1);
        } else if (gamepad1.right_bumper) {
            FLW.setPower(1);
            FRW.setPower(-1);
            BLW.setPower(1);
            BRW.setPower(-1);
        } else {
            disablePower();
        }
    }

    protected void disablePower() {
        FLW.setPower(0);
        FRW.setPower(0);
        BLW.setPower(0);
        BRW.setPower(0);
    }

    protected void enableEncoders() {
        FLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public boolean delay() {
        return System.currentTimeMillis() >= time + 250;
    }

    public boolean delay(long duration) {
        return System.currentTimeMillis() >= time + duration;
    }

    public void setTime() {
        time = System.currentTimeMillis();
    }
}