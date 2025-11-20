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
public class MaliceAndCondescensionGRACE extends Movable implements LimelightTags { // robot #22335

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
    private static final double SWEEP_SPEED = 0.01;

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
        swivelTurret = new DoubleSwitchedServo(swivelTurretServo, .07, .57);
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

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        FLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretStop = true;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("CURRENT TARGETED ID", targetedID);
            telemetry.addData("Outtake Encoder", outtakeMotor.getCurrentPosition());
            omnidirectionalMovement(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            turn();

            //if (tracking) PrincessEyes();
            detectTag(limelight, telemetry);
            updateSweep();
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

            if (gamepad1.left_bumper) {
                hoodServo.setPosition(hoodServo.getPosition() - .01);
            } else if (gamepad1.right_bumper && hoodServo.getPosition() <= .8) {
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

            if (gamepad1.share && delay(500)) {
                //tracking = !tracking;
                startSweep();
                time = System.currentTimeMillis();
            }

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

            if (gamepad1.dpad_right) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - 0.004);
            } else if (gamepad1.dpad_left) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + 0.004);
            }

            if (intakeToggle) {
                boolean LUp = wiperL.getPosition() == wipersL.getSecondaryPos();
                boolean RUp = wiperR.getPosition() == wipersR.getSecondaryPos();
                // these two variables not needed but it stops working whenever I remove them so uhhhhhh
                boolean gatewayL = gatewayServo.getPosition() == .73;
                boolean gatewayR = gatewayServo.getPosition() == .26;
                if ((LUp && gatewayL) || (RUp && gatewayR) || (!LUp && !RUp)) {
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


            // gamepad 2
            if (gamepad2.left_trigger >= .5 && delay(1001)) {
                liftLeftWiper();
                time = System.currentTimeMillis();
            } else if (gamepad2.right_trigger >= .5 && delay(1001)) {
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

            if (gamepad2.dpad_right) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - 0.004);
            } else if (gamepad2.dpad_left) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + 0.004);
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

    private void startSweep() {
        turretStop = false;
        sweeping = true;

        double left = swivelTurret.getPrimaryPos();
        double right = swivelTurret.getSecondaryPos();
        double pos = swivelTurretServo.getPosition();

        double distLeft = Math.abs(pos - left);
        double distRight = Math.abs(pos - right);

        // sweep toward whichever side is farther away
        sweepDirection = (distLeft >= distRight) ? -1 : 1; // ask gpt I didn't write this one :P
    }

    private void updateSweep() {
        if (!sweeping) return;

        // if tag20() set turretStop = true → stop immediately
        if (turretStop) {
            sweeping = false;

            return;
        }

        double pos = swivelTurretServo.getPosition();
        double left = swivelTurret.getPrimaryPos();
        double right = swivelTurret.getSecondaryPos();

        if (sweepDirection < 0) { // sweeping left
            if (pos > left) {
                swivelTurretServo.setPosition(pos - SWEEP_SPEED);
            } else sweeping = false;
        }
        else { // sweeping right
            if (pos < right) {
                swivelTurretServo.setPosition(pos + SWEEP_SPEED);
            } else sweeping = false;
        }
    }


    @Override
    public void tag20() {
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

    public void nothing() {}

    @Override
    protected void turn() {
        final double POWER = .75;
        final double SPEED = 0.0025;
        if (gamepad1.right_stick_x < -.3) { // turn left
            FLW.setPower(-POWER);
            FRW.setPower(POWER);
            BLW.setPower(-POWER);
            BRW.setPower(POWER);
            if (swivelTurretServo.getPosition() >= .07) swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
        } else if (gamepad1.right_stick_x > .3) { // turn right
            FLW.setPower(POWER);
            FRW.setPower(-POWER);
            BLW.setPower(POWER);
            BRW.setPower(-POWER);
            if (swivelTurretServo.getPosition() <= .57) swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
        } else if (gamepad1.right_stick_x >= -.3 && gamepad1.right_stick_x <= .3
                && gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
            disablePower();
        }
    }
}