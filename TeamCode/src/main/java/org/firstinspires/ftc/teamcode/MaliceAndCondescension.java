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

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // april tags
        limelight.start();

        swivelTurretServo = hardwareMap.get(Servo.class, "swivelTurret");


        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtake");
        intakeToggle = false;
        outtakeToggle = false;

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

            moveWheels(gamepad1.left_stick_x, gamepad1.left_stick_y);
            strafe();
            turretTracking();

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

            if (gamepad1.x) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

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
                outtakeMotor.setPower(1);
            } else {
                outtakeMotor.setPower(0);
            }

            telemetry.update();
        }
    }

    private void turretTracking() {
        // (looking @ the bot from the back) .03 limit for camera looking at left, .425 on the other
        boolean isContained = (swivelTurretServo.getPosition() >= .3 && swivelTurretServo.getPosition() <= .425);

        double tx = getTX(limelight);
        if (!Double.isNaN(tx)) {
            if (tx >= 1 && isContained) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - 0.0005);
            } else if (tx <= -1 && isContained) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + 0.0005);
            }
            telemetry.addData("I see the tag at", getTX(limelight));
            telemetry.addData("The tag I see is", detectTag(limelight, telemetry));
        } else {
            telemetry.addData("NaN", true);
        }
        telemetry.addData("Turret Servo Position", swivelTurretServo.getPosition());
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
}