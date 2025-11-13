package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@TeleOp
public class MaliceAndCondescension extends Movable implements LimelightTags { // robot #22335

    private static Limelight3A limelight;
    private static Servo swivelTurretServo;
    private static DcMotor intakeMotor;
    private static DcMotorEx outtakeMotor;
    private static boolean intakeToggle, outtakeToggle;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;
    private static int targetedID;
    private static double targetRPM;
    private static boolean turnLeft;
    private static boolean tracking;
    private static Servo hoodServo;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        swivelTurretServo = hardwareMap.get(Servo.class, "swivelTurret");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");

        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        gateways = new DoubleSwitchedServo(gatewayServo, .26, .73); // .26: right side (when looking at the back)

        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);

        hoodServo = hardwareMap.get(Servo.class, "hood");

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);

        reset();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("CURRENT TARGETED ID", targetedID);
            telemetry.addData("Outtake Encoder", outtakeMotor.getCurrentPosition());
            moveWheels(gamepad1.left_stick_x, gamepad1.left_stick_y);
            strafe();

            if (tracking) LeBotsEyes();

            if (gamepad1.b && delay()) {
                gateways.quickSwitch();
                time = System.currentTimeMillis();
            } else if (gamepad1.left_trigger >= .5 && delay(1001)) {
                liftLeftWiper();
                time = System.currentTimeMillis();
            } else if (gamepad1.right_trigger >= .5 && delay(1001)) {
                liftRightWiper();
                time = System.currentTimeMillis();
            }

            if (gamepad1.y && delay()) {
                intakeToggle = !intakeToggle;
                time = System.currentTimeMillis();
            } else if (gamepad1.a && delay()) {
                outtakeToggle = !outtakeToggle;
                time = System.currentTimeMillis();
            }

            if (gamepad1.right_stick_y > 0.3) {
                hoodServo.setPosition(hoodServo.getPosition() - .01);
            } else if (gamepad1.right_stick_y < -0.3) {
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

            if (gamepad1.share && delay()) {
                tracking = !tracking;
                time = System.currentTimeMillis();
            }

            if (gamepad1.x) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad1.dpad_up && delay()) {
                targetRPM += 50;
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && delay()) {
                targetRPM -= 50;
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
            } else {
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

            telemetry.update();
        }
    }

    private void liftRightWiper() {
        new Thread(() -> {
            try {
                wipersR.secondaryPos();
                Thread.sleep(1000);
                wipersR.primaryPos();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }).start();
    }

    private void liftLeftWiper() {
        new Thread(() -> {
            try {
                wipersL.secondaryPos();
                Thread.sleep(1000);
                wipersL.primaryPos();
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }).start();
    }

    private double getTargetTicksPerSec(double ticksPerRev, double targetRPM) {
        return (ticksPerRev * targetRPM) / 60;
    }
    private void LeBotsEyes() {
        final double SPEED = 0.002;
        double tx = getTX(limelight);

        telemetry.addData("Tag X", getTX(limelight));
        telemetry.addData("Turret Servo Position", swivelTurretServo.getPosition());

        if ((Double.isNaN(tx) || tx >= 3 || tx <= -3) && detectTag(limelight, telemetry) != targetedID) {
            if (turnLeft) { // move towards .03, left @ back)
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
                if (swivelTurretServo.getPosition() <= .03) turnLeft = false; // turns right
            } else {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
                if (swivelTurretServo.getPosition() >= .425) turnLeft = true;
            }
        }
    }

    private void reset() {
        limelight.pipelineSwitch(0); // april tags
        limelight.start();
        targetedID = 20;
        tracking = true;
        swivelTurretServo.setPosition(.1975);
        turnLeft = false;
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeToggle = false;
        outtakeToggle = false;
        targetRPM = 300;
        wipersL.primaryPos();
        wipersR.primaryPos();
        hoodServo.setPosition(0);
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
    protected void strafe() {
        final double POWER = .75;
        final double SPEED = 0.0025;
        if (gamepad1.left_bumper) {
            FLW.setPower(-POWER);
            FRW.setPower(POWER);
            BLW.setPower(-POWER);
            BRW.setPower(POWER);
            if (swivelTurretServo.getPosition() >= .03) swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
        } else if (gamepad1.right_bumper) {
            FLW.setPower(POWER);
            FRW.setPower(-POWER);
            BLW.setPower(POWER);
            BRW.setPower(-POWER);
            if (swivelTurretServo.getPosition() <= .425) swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
        } else {
            disablePower();
        }
    }
}