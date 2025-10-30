package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

import java.util.List;

@TeleOp
public class MaliceAndCondescension extends Movable implements LimelightColor, LimelightTags { // robot #22335

    private static Limelight3A limelight;
    private static Servo swivelServo;
    private static List<LLResultTypes.FiducialResult> results;
    private static DcMotor intakeMotor;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //swivelServo = hardwareMap.get(Servo.class, "swivelServo");

        //limelight.pipelineSwitch(0); // april tags
        //limelight.start();

        //intakeMotor = hardwareMap.get(DcMotor.class, "intake"); // placeholder
        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        gateways = new DoubleSwitchedServo(gatewayServo, .23, .76);
        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);

        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            moveWheels(gamepad1.left_stick_x, gamepad1.left_stick_y);

            if (gamepad1.b && delay()) {
                gateways.quickSwitch();
                time = System.currentTimeMillis();
            } else if (gamepad1.left_trigger >= .5 && delay(600)) { // swap names of wiper variables, reversed
                new Thread(() -> {
                    try {
                        wipersL.secondaryPos();
                        Thread.sleep(500);
                        wipersL.primaryPos();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }).start();
                time = System.currentTimeMillis();
            } else if (gamepad1.right_trigger >= .5 && delay(600)) {
                new Thread(() -> {
                    try {
                        wipersR.secondaryPos();
                        Thread.sleep(500);
                        wipersR.primaryPos();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }).start();
                time = System.currentTimeMillis();
            }

            if (gamepad1.left_bumper) {
                FLW.setPower(-1);
                FRW.setPower(1);
                BLW.setPower(-1);
                BRW.setPower(1);
            } else if (gamepad1.right_bumper) {
                FLW.setPower(1);
                FRW.setPower(-1);
                BLW.setPower(1);
                BRW.setPower(-1);
            } else {
                disablePower();
            }

            /*else if (gamepad1.options && delay()) {
                intakeMotor.setPower(1);
                time = System.currentTimeMillis();
            } else {
                intakeMotor.setPower(0);
            }*/

            telemetry.update();
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
    public void green() {

    }

    @Override
    public void purple() {

    }

    /* code for limelight

    if (gamepad1.x) { // reset swivel servo position
                swivelServo.setPosition(1);
            }

    if (gamepad1.a) {

        telemetry.addData("Servo Position", swivelServo.getPosition());

        int tag = detectTag(limelight, telemetry);
        if (tag == -1
            //&& swivelServo.getPosition() < 1.0
        ) {
            //double nextPos = swivelServo.getPosition() + 0.0005;
            //swivelServo.setPosition(nextPos);
            telemetry.addData("Hello", "It should be rotating");
        } else {
            telemetry.addData("Tag detected", tag);
        }
        //telemetry.addData("Servo Position", swivelServo.getPosition());

    } else if (gamepad1.b) { // reset swivel servo position
        swivelServo.setPosition(0);
        swivelServo.setDirection(Servo.Direction.FORWARD);
    }
    */
}