package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@TeleOp
public class MaliceAndCondescension extends Movable implements LimelightTags { // robot #22335

    private static Limelight3A limelight;
    private static Servo swivelTurretServo;
    private static DcMotor intakeMotor, outtakeMotor;
    private static boolean intakeToggle, outtakeToggle;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;
    private static int targetedID;
    private static double OUTTAKE_POWER;
    private static boolean turn;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // april tags
        limelight.start();
        targetedID = 20;

        swivelTurretServo = hardwareMap.get(Servo.class, "swivelTurret");
        swivelTurretServo.setPosition(.1975);
        turn = false;

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtake");
        intakeToggle = false;
        outtakeToggle = false;
        OUTTAKE_POWER = .5;

        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        gateways = new DoubleSwitchedServo(gatewayServo, .26, .73); // .26: right side (when looking at the back)

        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);
        wipersL.primaryPos();
        wipersR.primaryPos();

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("CURRENT TARGETED ID", targetedID);

            moveWheels(gamepad1.left_stick_x, gamepad1.left_stick_y);
            strafe();
            LeBotsEyes();

            if (gamepad1.b && delay()) {
                gateways.quickSwitch();
                time = System.currentTimeMillis();
            } else if (gamepad1.left_trigger >= .5 && delay(1001)) {
                new Thread(() -> {
                    try {
                        wipersL.secondaryPos();
                        Thread.sleep(1000);
                        wipersL.primaryPos();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }).start();
                time = System.currentTimeMillis();
            } else if (gamepad1.right_trigger >= .5 && delay(1001)) {
                new Thread(() -> {
                    try {
                        wipersR.secondaryPos();
                        Thread.sleep(1000);
                        wipersR.primaryPos();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }).start();
                time = System.currentTimeMillis();
            }

            if (gamepad1.y && delay()) {
                intakeToggle = !intakeToggle;
                time = System.currentTimeMillis();
            } else if (gamepad1.a && delay()) {
                outtakeToggle = !outtakeToggle;
                time = System.currentTimeMillis();
            }

            if (gamepad1.options && delay()) {
                if (targetedID == 20) {
                    targetedID = 24;
                } else if (targetedID == 24) {
                    targetedID = 20;
                }
                time = System.currentTimeMillis();
            }

            if (gamepad1.x) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad1.dpad_up && delay() && OUTTAKE_POWER < 1) {
                OUTTAKE_POWER += .1;
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && delay() && OUTTAKE_POWER > 0) {
                OUTTAKE_POWER -= .1;
                time = System.currentTimeMillis();
            }

            telemetry.addData("Outtake power", OUTTAKE_POWER);

            if (intakeToggle) {
                boolean LUp = wiperL.getPosition() == wipersL.getSecondaryPos();
                boolean RUp = wiperR.getPosition() == wipersR.getSecondaryPos();
                boolean gatewayL = gatewayServo.getPosition() == .73;
                boolean gatewayR = gatewayServo.getPosition() == .26;
                if ((LUp && gatewayL) || (RUp && gatewayR) || (!LUp && !RUp)) {
                    intakeMotor.setPower(1);
                }
            } else {
                intakeMotor.setPower(0);
            }

            if (outtakeToggle) {
                outtakeMotor.setPower(OUTTAKE_POWER);
            } else {
                outtakeMotor.setPower(0);
            }

            telemetry.update();
        }
    }

    private void LeBotsEyes() {
        final double SPEED = 0.001;
        double tx = getTX(limelight);

        if (!Double.isNaN(tx)) {
            telemetry.addData("Tag X", getTX(limelight));
            telemetry.addData("Turret Servo Position", swivelTurretServo.getPosition());

            if (detectTag(limelight, telemetry) == targetedID) {
                if (tx >= 3 && swivelTurretServo.getPosition() >= .03) {
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
                } else if (tx <= -3 && swivelTurretServo.getPosition() <= .425) {
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
                }
            }
        } else { // constantly rotate turret until it sees the tag (dunno if it works yet)
            if (turn) { // move towards .03
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
                if (swivelTurretServo.getPosition() <= .03) turn = false;
            } else {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
                if (swivelTurretServo.getPosition() >= .425) turn = true;
            }
        }

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