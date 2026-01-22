package org.firstinspires.ftc.teamcode.KIDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;

@TeleOp
public class THE_KIDS extends LinearOpMode implements LimelightTags { // robot #22335

    private static Servo swivelTurretServo;
    private static DcMotor intakeMotor;
    private static DcMotorEx outtakeMotor;
    private static boolean intakeToggle, outtakeToggle;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;
    private static DoubleSwitchedServo swivelTurret;
    private static double targetRPM;
    private static Servo hoodServo;

    private static long time;

    @Override
    public void runOpMode() throws InterruptedException {
        swivelTurretServo = hardwareMap.get(Servo.class, "swivelTurret");
        swivelTurret = new DoubleSwitchedServo(swivelTurretServo, .1, .54);

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

        hoodServo = hardwareMap.get(Servo.class, "hood");

        time = System.currentTimeMillis();

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        wipersL.primaryPos();
        wipersR.primaryPos();
        swivelTurretServo.setPosition(.1975);
        hoodServo.setPosition(0);

        while (opModeIsActive()) {

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

            if (gamepad1.dpad_up) {
                hoodServo.setPosition(hoodServo.getPosition() - .01);
            } else if (gamepad1.dpad_left && hoodServo.getPosition() <= .8) {
                hoodServo.setPosition(hoodServo.getPosition() + .01);
            }

            if (gamepad1.dpad_up && delay(100)) {
                targetRPM += 25;
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && delay(100)) {
                targetRPM -= 25;
                time = System.currentTimeMillis();
            }

            if (gamepad1.right_bumper && swivelTurretServo.getPosition() > swivelTurret.getPrimaryPos()) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - 0.004);
            } else if (gamepad1.left_bumper) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + 0.004);
            }

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

            telemetry.addData("Target RPM", Math.abs(targetRPM));
            telemetry.addData("Hood Position", hoodServo.getPosition());

            telemetry.addLine();
            telemetry.addLine("EXTRA INFO (IGNORE)");
            telemetry.addLine();

            telemetry.addData("Outtake Encoder", Math.abs(outtakeMotor.getCurrentPosition()));
            telemetry.addData("Swivel Turret Position", swivelTurretServo.getPosition());
            telemetry.addData("Measured RPM", Math.abs(outtakeMotor.getVelocity() * 60.0 / 28));
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
    public boolean delay() {
        return System.currentTimeMillis() >= time + 250;
    }

    public boolean delay(long duration) {
        return System.currentTimeMillis() >= time + duration;
    }

}

