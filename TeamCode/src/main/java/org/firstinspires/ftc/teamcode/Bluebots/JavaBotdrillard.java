package org.firstinspires.ftc.teamcode.Bluebots;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

public abstract class JavaBotdrillard extends Movable {
    protected static DcMotor swivelTurretMotor;
    protected static DcMotor intakeMotor;
    protected static DcMotorEx outtakeMotor;
    protected static boolean intakeToggle, outtakeToggle;
    protected static Servo gatewayServo;
    protected static DoubleSwitchedServo gateways;
    protected static Servo wiperL, wiperR;
    protected static DoubleSwitchedServo wipersL, wipersR;
    protected static DoubleSwitchedServo swivelTurret;
    protected static boolean turnLeft;
    protected static boolean tracking;
    protected static Servo hoodServo;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        tracking = true;
        turnLeft = false;

        swivelTurretMotor = hardwareMap.get(DcMotor.class, "swivelTurret");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        intakeToggle = false;
        outtakeToggle = false;
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        gateways = new DoubleSwitchedServo(gatewayServo, .26, .73); // .26: right side (when looking at the back)
        gateways.secondaryPos();

        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);
        wipersL.primaryPos();
        wipersR.primaryPos();

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(0);

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        FLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(2, 0, 0, 13.2);
        outtakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    protected enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT,
        STRAFE_LEFT,
        STRAFE_RIGHT;
    }

    protected void move(Direction dir, int ms) {
        final double POWER = .2;
        switch (dir) {
            case FORWARD:
                FLW.setPower(POWER);
                FRW.setPower(POWER);
                BLW.setPower(POWER);
                BRW.setPower(POWER);
                break;
            case BACKWARD:
                FLW.setPower(-POWER);
                FRW.setPower(-POWER);
                BLW.setPower(-POWER);
                BRW.setPower(-POWER);
                break;
            case LEFT:
                FLW.setPower(-POWER);
                FRW.setPower(POWER);
                BLW.setPower(-POWER);
                BRW.setPower(POWER);
                break;
            case RIGHT:
                FLW.setPower(POWER);
                FRW.setPower(-POWER);
                BLW.setPower(POWER);
                BRW.setPower(-POWER);
                break;
            case STRAFE_LEFT:
                FLW.setPower(-POWER);
                FRW.setPower(POWER);
                BLW.setPower(POWER);
                BRW.setPower(-POWER);
                break;
            case STRAFE_RIGHT:
                FLW.setPower(POWER);
                FRW.setPower(-POWER);
                BLW.setPower(-POWER);
                BRW.setPower(POWER);
                break;
            default: break;
        }
        sleep(ms);
        disablePower();
        sleep(500);
    }

    protected void move(Direction dir, int ms, final double POWER) {
        switch (dir) {
            case FORWARD:
                FLW.setPower(POWER);
                FRW.setPower(POWER);
                BLW.setPower(POWER);
                BRW.setPower(POWER);
                break;
            case BACKWARD:
                FLW.setPower(-POWER);
                FRW.setPower(-POWER);
                BLW.setPower(-POWER);
                BRW.setPower(-POWER);
                break;
            case LEFT:
                FLW.setPower(-POWER);
                FRW.setPower(POWER);
                BLW.setPower(-POWER);
                BRW.setPower(POWER);
                break;
            case RIGHT:
                FLW.setPower(POWER);
                FRW.setPower(-POWER);
                BLW.setPower(POWER);
                BRW.setPower(-POWER);
                break;
            case STRAFE_LEFT:
                FLW.setPower(-POWER);
                FRW.setPower(POWER);
                BLW.setPower(POWER);
                BRW.setPower(-POWER);
                break;
            case STRAFE_RIGHT:
                FLW.setPower(POWER);
                FRW.setPower(-POWER);
                BLW.setPower(-POWER);
                BRW.setPower(POWER);
                break;
            default: break;
        }
        sleep(ms);
        disablePower();
        sleep(500);
    }

    protected void liftRightWiper() {
        intakeMotor.setPower(0);
        wipersR.secondaryPos();
        sleep(1000);
        wipersR.primaryPos();
        intakeMotor.setPower(1);
    }

    protected void liftLeftWiper() {
        intakeMotor.setPower(0);
        wipersL.secondaryPos();
        sleep(1000);
        wipersL.primaryPos();
        intakeMotor.setPower(1);
    }
}