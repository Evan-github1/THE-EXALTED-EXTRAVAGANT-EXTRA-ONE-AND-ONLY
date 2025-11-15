package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@Autonomous
public class EcstasyOfAutomation extends Movable implements LimelightTags {
    private static Limelight3A limelight;
    private static Servo swivelTurretServo;
    private static DcMotor intakeMotor;
    private static DcMotorEx outtakeMotor;
    private static boolean intakeToggle, outtakeToggle;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;
    private static boolean turnLeft;
    private static boolean tracking;
    private static Servo hoodServo;


    @Override
    public void runOpMode() throws InterruptedException{
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

        reset();

        waitForStart();

        // detect for tag
        outtakeMotor.setVelocity(getTargetTicksPerSec(28, 2200));
        intakeMotor.setPower(1);
        hoodServo.setPosition(.76);
        sleep(2000);
        while (detectTag(limelight, telemetry) != 20
        && getTX(limelight) <= -3 && getTX(limelight) >= 3) {
            PrincessEyes();
        }
        liftRightWiper();
        sleep(2000);
        liftLeftWiper();
        sleep(2000);
        liftRightWiper();
        sleep(2000);
    }

    private double getTargetTicksPerSec(double ticksPerRev, double targetRPM) {
        return (ticksPerRev * targetRPM) / 60;
    }

    private void PrincessEyes() {

        // .76 hood
        // 2200 rpm outtake power
        final double SPEED = 0.002;
        double tx = getTX(limelight);
        int ID = detectTag(limelight, telemetry);

        telemetry.addData("Tag ID", ID);
        telemetry.addData("Tag X", tx);
        telemetry.addData("Turret Servo Position", swivelTurretServo.getPosition());

        if (ID != 20) {
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

    private void liftRightWiper() {
        try {
            wipersR.secondaryPos();
            Thread.sleep(1000);
            wipersR.primaryPos();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void liftLeftWiper() {
        try {
            wipersL.secondaryPos();
            Thread.sleep(1000);
            wipersL.primaryPos();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void reset() {
        limelight.pipelineSwitch(0); // april tags
        limelight.start();
        tracking = true;
        swivelTurretServo.setPosition(.1975);
        turnLeft = false;
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeToggle = false;
        outtakeToggle = false;
        wipersL.primaryPos();
        wipersR.primaryPos();
        hoodServo.setPosition(0);
        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        FLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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