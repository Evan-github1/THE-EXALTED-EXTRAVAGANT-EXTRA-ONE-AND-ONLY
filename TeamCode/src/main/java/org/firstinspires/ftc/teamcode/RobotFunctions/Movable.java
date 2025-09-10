package org.firstinspires.ftc.teamcode.RobotFunctions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// TODO: inherit this class to be able to drive
public abstract class Movable extends LinearOpMode {
    static protected DcMotor FLW;
    static protected DcMotor BLW;
    static protected DcMotor FRW;
    static protected DcMotor BRW;
    static protected long time;

    static protected double angle, desVol, vx, vy, v1, v2, max;

    static {
        time = System.currentTimeMillis();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FLW = hardwareMap.get(DcMotor.class, "FLW");
        BLW = hardwareMap.get(DcMotor.class, "BLW");
        FRW = hardwareMap.get(DcMotor.class, "FRW");
        BRW = hardwareMap.get(DcMotor.class, "BRW");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
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

        FLW.setDirection(DcMotorSimple.Direction.REVERSE);

        FLW.setDirection(DcMotor.Direction.FORWARD);
        BLW.setDirection(DcMotor.Direction.FORWARD);
        FRW.setDirection(DcMotor.Direction.REVERSE);
        BRW.setDirection(DcMotor.Direction.REVERSE);

        FLW.setPower(v1);
        FRW.setPower(v2);
        BRW.setPower(v1);
        BLW.setPower(v2);
    }

    protected abstract void updatePhoneConsole();
}