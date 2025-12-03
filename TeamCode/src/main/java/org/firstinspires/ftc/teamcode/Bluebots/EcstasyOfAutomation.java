package org.firstinspires.ftc.teamcode.Bluebots;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

public abstract class EcstasyOfAutomation extends Movable implements LimelightTags {
    protected static Limelight3A limelight;
    protected static Servo swivelTurretServo;
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

    protected static boolean sweepInit;
    protected static boolean sweepActive;
    protected static double sweepTarget;
    protected static final int targetedID = 20;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0); // april tags
        tracking = true;
        turnLeft = false;

        swivelTurretServo = hardwareMap.get(Servo.class, "swivelTurret");
        swivelTurret = new DoubleSwitchedServo(swivelTurretServo, .09, .55);

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

        waitForStart();
    }

    protected enum Direction {
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT;
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
            default: break;
        }
        sleep(ms);
        disablePower();
        sleep(500);
    }

    protected double getTargetTicksPerSec(double ticksPerRev, double targetRPM) {
        return (ticksPerRev * targetRPM) / 60;
    }//0.00005

    protected void PrincessEyes(int ID) {
        final double SPEED = 0.00005;
        final double CENTER_TOLERANCE = 2.0;   // Stop centering when |tx| < 2 degrees

        // 1. SWEEP UNTIL TAG FOUND
        while (opModeIsActive() && detectTag(limelight, telemetry) != ID) {

            if (turnLeft) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
                if (swivelTurretServo.getPosition() <= .03) turnLeft = false;
            } else {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
                if (swivelTurretServo.getPosition() >= .425) turnLeft = true;
            }

            telemetry.addData("Searching", true);
            telemetry.update();
        }

        // 2. CENTER ONCE (STOP AS SOON AS CENTERED)
        if (opModeIsActive() && detectTag(limelight, telemetry) == ID) {
            while (opModeIsActive()) {
                double tx = getTX(limelight);

                if (Math.abs(tx) <= CENTER_TOLERANCE) {
                    telemetry.addData("Centered", true);
                    telemetry.update();
                    break;
                }

                if (tx < -CENTER_TOLERANCE) {
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
                } else if (tx > CENTER_TOLERANCE) {
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
                }

                telemetry.addData("Centering", tx);
                telemetry.update();
            }
        }
    }

    protected void PrincessEyesv2(int targetedID) { // same as teleop
        final double SPEED = 0.002;
        double tx = getTX(limelight);
        int ID = detectTag(limelight, telemetry);
        telemetry.addData("Tag ID", ID);
        telemetry.addData("Tag X", tx);
        telemetry.addData("Turret Servo Position", swivelTurretServo.getPosition());

        if (ID != targetedID) {
            if (turnLeft) { // move towards .03, left @ back
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
                if (swivelTurretServo.getPosition() <= .03) turnLeft = false; // switch direction
            } else {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
                if (swivelTurretServo.getPosition() >= .425) turnLeft = true;
            }
        } else {
            if (tx <= -3 && swivelTurretServo.getPosition() >= .425) {
                // move turret left
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
            } else if (tx >= 3 && swivelTurretServo.getPosition() <= .03) {
                // move turret right
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
            }
        }
    }

    protected void PrincessEyesv3() {
        double leftPos = swivelTurret.getPrimaryPos();
        double rightPos = swivelTurret.getSecondaryPos();
        double step = 0.003;  // sweep speed per cycle
        double tolerance = 2.0; // degrees — tune this for how centered you want

        // Initialize sweep state on first run
        if (!sweepInit) {
            sweepInit = true;
            sweepActive = false;
            sweepTarget = leftPos;
        }

        // Button toggles sweep direction
        if (gamepad1.share && !sweepActive && delay()) {
            sweepActive = true;

            double current = swivelTurretServo.getPosition();

            // Choose opposite direction
            if (Math.abs(current - leftPos) < 0.1) {
                sweepTarget = rightPos;
            } else {
                sweepTarget = leftPos;
            }

            time = System.currentTimeMillis();
        }

        // Perform sweep movement
        if (sweepActive) {

            // Check for the desired AprilTag
            int tag = detectTag(limelight, telemetry);

            if (tag == targetedID) {

                double tx = getTX(limelight); // your method

                if (!Double.isNaN(tx)) {

                    telemetry.addData("TagSeen", tag);
                    telemetry.addData("tx", tx);

                    // If tag is centered -> stop turret
                    if (Math.abs(tx) < tolerance) {
                        sweepActive = false;
                        telemetry.addLine("Turret centered on tag " + targetedID);
                        return;
                    }

                    // Otherwise: keep sweeping until centered
                }
            }

            double current = swivelTurretServo.getPosition();
            double next;

            // Move incrementally toward target
            if (current < sweepTarget) {
                next = Math.min(current + step, sweepTarget);
            } else {
                next = Math.max(current - step, sweepTarget);
            }

            swivelTurretServo.setPosition(next);

            // Stop if we've reached the physical target
            if (Math.abs(next - sweepTarget) < 0.005) {
                sweepActive = false;
            }
        }

        telemetry.addData("TurretPos", swivelTurretServo.getPosition());
        telemetry.addData("SweepActive", sweepActive);
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

    @Override
    public void tag20() {

    }

    @Override
    public void tag21() {

    }

    @Override
    public void tag22() {

    }

    @Override
    public void tag23() {

    }

    @Override
    public void tag24() {

    }
}