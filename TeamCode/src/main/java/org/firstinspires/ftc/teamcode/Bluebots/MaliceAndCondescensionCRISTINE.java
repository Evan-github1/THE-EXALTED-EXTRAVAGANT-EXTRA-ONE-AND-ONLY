package org.firstinspires.ftc.teamcode.Bluebots;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
@TeleOp
public class MaliceAndCondescensionCRISTINE extends Movable implements LimelightTags { // robot #22335

    private static Limelight3A limelight;
    private static Servo swivelTurretServo;
    private static DcMotor intakeMotor;
    private static DcMotorEx outtakeMotor;
    private static boolean intakeToggle, outtakeToggle;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;
    private static DoubleSwitchedServo swivelTurret;
    private static int targetedID;
    private static double targetRPM;
    private static boolean turnLeft;
    private static boolean tracking;
    private static Servo hoodServo;
    private static volatile boolean turretStop;
    private static boolean sweeping = false;
    private static double sweepDirection = 0; // +1 = sweep right, -1 = sweep left
    private static final double SWEEP_SPEED = 0.005;
    private static boolean sweepInit;
    private static boolean sweepActive;
    private static double sweepTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0); // april tags
        targetedID = 20;
        tracking = true;
        turnLeft = false;

        swivelTurretServo = hardwareMap.get(Servo.class, "swivelTurret");
        swivelTurret = new DoubleSwitchedServo(swivelTurretServo, .09, .55);
        swivelTurretServo.setPosition(.1975);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        intakeToggle = false;
        outtakeToggle = false;
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetRPM = 1450;

        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        gateways = new DoubleSwitchedServo(gatewayServo, .26, .73); // .26: right side (when looking at the back)

        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);
        wipersL.primaryPos();
        wipersR.primaryPos();

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(0);

        FLW.setDirection(DcMotor.Direction.FORWARD);
        BLW.setDirection(DcMotor.Direction.FORWARD);
        FRW.setDirection(DcMotor.Direction.REVERSE);
        BRW.setDirection(DcMotor.Direction.REVERSE);
        FLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretStop = true;

        sweepInit = false;
        sweepActive = false;
        sweepTarget = 0.07;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("CURRENT TARGETED ID", targetedID);
            telemetry.addData("Outtake Encoder", outtakeMotor.getCurrentPosition());
            omnidirectionalMovement(gamepad1.left_stick_x, gamepad1.left_stick_y);
            turn();

            //if (tracking) PrincessEyes();
            int id = detectTag(limelight, telemetry);
            telemetry.addData("Turret Stop", turretStop);

            telemetry.addData("Swivel Turret Position", swivelTurretServo.getPosition());
            if (gamepad1.b && delay()) {
                gateways.quickSwitch();
                time = System.currentTimeMillis();
            }

            if (gamepad1.left_trigger >= .5 && delay(1001)) {
                liftRightWiper();
                time = System.currentTimeMillis();
            } else if (gamepad1.right_trigger >= .5 && delay(1001)) {
                liftLeftWiper();
                time = System.currentTimeMillis();
            }

            if (gamepad1.y && delay()) {
                intakeToggle = !intakeToggle;
                time = System.currentTimeMillis();
            } else if (gamepad1.a && delay()) {
                outtakeToggle = !outtakeToggle;
                time = System.currentTimeMillis();
            }

            if (gamepad1.right_stick_y > .3) {
                hoodServo.setPosition(hoodServo.getPosition() - .01);
            } else if (gamepad1.right_stick_y < -.3 && hoodServo.getPosition() <= .8) {
                hoodServo.setPosition(hoodServo.getPosition() + .01);
            }
            telemetry.addData("Hood Position", hoodServo.getPosition());

            if (gamepad1.options && delay()) {
                if (targetedID == 20) {
                    targetedID = 24;
                } else if (targetedID == 24) {
                    targetedID = 20;
                }
                time = System.currentTimeMillis();
            }

            PrincessEyesv3(); // uses option button

            if (gamepad1.x) {
                intakeMotor.setPower(-1);
            }

            if (gamepad1.dpad_up && delay() && targetRPM <= 2300) {
                if (targetRPM == 1600) {
                    targetRPM = 2100;
                } else {
                    targetRPM += 50;
                }
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && delay() && targetRPM >= 1450) {
                if (targetRPM == 2100) {
                    targetRPM = 1600;
                } else {
                    targetRPM -= 50;
                }
                time = System.currentTimeMillis();
            }

            if (gamepad1.dpad_right && swivelTurretServo.getPosition() < swivelTurret.getSecondaryPos()) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - 0.004);
            } else if (gamepad1.dpad_left && swivelTurretServo.getPosition() > swivelTurret.getPrimaryPos()) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + 0.004);
            }


            if (gamepad1.leftStickButtonWasPressed() && delay()) {
                if (gatewayServo.getPosition() < .5) {
                    new Thread(() -> {
                        wipersL.secondaryPos();
                        sleep(1000);
                        wipersL.primaryPos();

                        sleep(500);

                        wipersR.secondaryPos();
                        sleep(1000);
                        wipersR.primaryPos();

                        sleep(500);

                        wipersL.secondaryPos();
                        sleep(1000);
                        wipersL.primaryPos();
                    }).start();
                } else if (gatewayServo.getPosition() > .5) {
                    new Thread(() -> {
                        wipersR.secondaryPos();
                        sleep(1000);
                        wipersR.primaryPos();

                        sleep(500);

                        wipersL.secondaryPos();
                        sleep(1000);
                        wipersL.primaryPos();

                        sleep(500);

                        wipersR.secondaryPos();
                        sleep(1000);
                        wipersR.primaryPos();

                    }).start();
                }
                time = System.currentTimeMillis();
            }

            telemetry.addData("Gateway Position", gatewayServo.getPosition());

            if (intakeToggle) {
                boolean LUp = wiperL.getPosition() == wipersL.getSecondaryPos();
                boolean RUp = wiperR.getPosition() == wipersR.getSecondaryPos();

                if (LUp || RUp) {
                    intakeMotor.setPower(0);
                } else {
                    intakeMotor.setPower(1);
                }
            } else if (!gamepad1.x || !intakeToggle) {
                intakeMotor.setPower(0);
            }

            if (outtakeToggle) {
                outtakeMotor.setVelocity(getTargetTicksPerSec(28, targetRPM));
            } else {
                outtakeMotor.setVelocity(0);
            }
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Measured RPM", outtakeMotor.getVelocity() * 60.0 / 28);
            telemetry.addData("Ticks/sec", outtakeMotor.getVelocity());

            // GAMEPAD 2
            if (gamepad2.right_trigger >= .5 && delay(1001)) {
                liftLeftWiper();
                time = System.currentTimeMillis();
            } else if (gamepad2.left_trigger >= .5 && delay(1001)) {
                liftRightWiper();
                time = System.currentTimeMillis();
            }
            if (gamepad2.b && delay()) {
                gateways.quickSwitch();
                time = System.currentTimeMillis();
            }

            if (gamepad2.right_stick_y > 0.3) {
                hoodServo.setPosition(hoodServo.getPosition() - .01);
            } else if (gamepad2.right_stick_y < -0.3 && hoodServo.getPosition() <= .8) {
                hoodServo.setPosition(hoodServo.getPosition() + .01);
            }

            if (gamepad2.dpad_right && swivelTurretServo.getPosition() < swivelTurret.getSecondaryPos()) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - 0.004);
            } else if (gamepad2.dpad_left && swivelTurretServo.getPosition() > swivelTurret.getPrimaryPos()) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + 0.004);
            }

            // END OF GAMEPAD 2

            if (id == targetedID) {
                telemetry.addData("hi", "yes");
                turretStop = true;
                double tx = getTX(limelight);
                if (tx <= -3 && swivelTurretServo.getPosition() >= .57) {
                    // move turret left
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() - .0001);
                } else if (tx >= 3 && swivelTurretServo.getPosition() <= .07) {
                    // move turret right
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() + .0001);
                }
            }

            telemetry.update();
        }
    }

    private void liftRightWiper() {
        new Thread(() -> {
            wipersR.secondaryPos();
            sleep(1000);
            wipersR.primaryPos();
        }).start();
    }

    private void liftLeftWiper() {
        new Thread(() -> {
            wipersL.secondaryPos();
            sleep(1000);
            wipersL.primaryPos();
        }).start();
    }

    private double getTargetTicksPerSec(double ticksPerRev, double targetRPM) {
        return (ticksPerRev * targetRPM) / 60;
    }

    private void PrincessEyes() { // it works, if you're not Evan don't touch it or I will come for you
        final double SPEED = 0.002;
        double tx = getTX(limelight);
        int ID = detectTag(limelight, telemetry);
        telemetry.addData("Tag ID", ID);
        telemetry.addData("Tag X", tx);
        telemetry.addData("Turret Servo Position", swivelTurretServo.getPosition());

        if (ID != targetedID) {
            if (turnLeft) { // move towards .07, left @ back
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
                if (swivelTurretServo.getPosition() <= .07) turnLeft = false; // switch direction
            } else {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
                if (swivelTurretServo.getPosition() >= .57) turnLeft = true;
            }
        } else {
            if (tx <= -3 && swivelTurretServo.getPosition() >= .57) {
                // move turret left
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
            } else if (tx >= 3 && swivelTurretServo.getPosition() <= .07) {
                // move turret right
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
            }
        }
    }

    private void PrincessEyesv2() {
        turretStop = false;
        double distFromLeft = Math.abs(swivelTurretServo.getPosition() - swivelTurret.getPrimaryPos());
        double distFromRight = Math.abs(swivelTurretServo.getPosition() - swivelTurret.getSecondaryPos());
        if (distFromLeft >= distFromRight) {
            while (swivelTurretServo.getPosition() > swivelTurret.getPrimaryPos()) {
                if (!turretStop) {
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() - .0005);
                } else {
                    return;
                }
            }
        } else {
            while (swivelTurretServo.getPosition() < swivelTurret.getSecondaryPos()) {
                if (!turretStop) {
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() + .0005);
                } else {
                    return;
                }
            }
        }
    }

    public void PrincessEyesv3() {
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

    @Override
    public void nothing() {

    }

    @Override
    protected void turn() {
        final double POWER = .75;
        final double SPEED = 0.0025;
        if (gamepad1.right_bumper) { // turn right
            FLW.setPower(-POWER);
            FRW.setPower(POWER);
            BLW.setPower(-POWER);
            BRW.setPower(POWER);
            if (swivelTurretServo.getPosition() >= .07) swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
        } else if (gamepad1.left_bumper) { // turn left
            FLW.setPower(POWER);
            FRW.setPower(-POWER);
            BLW.setPower(POWER);
            BRW.setPower(-POWER);
            if (swivelTurretServo.getPosition() <= .57) swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
        } else if (gamepad1.rightBumperWasReleased() && gamepad1.leftBumperWasReleased()
                && gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
            disablePower();
        }
    }
}