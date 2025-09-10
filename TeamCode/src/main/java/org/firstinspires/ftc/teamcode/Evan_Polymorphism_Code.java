package org.firstinspires.ftc.teamcode;

// TODO: OLD AND OUTDATED

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LinearSlide;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;

@TeleOp
public class Evan_Polymorphism_Code extends Movable {

    private static Servo LPushServo, RPushServo;
    private static Servo twistingBottomServo;
    private static Servo LSwingServo, RSwingServo;
    private static Servo bottomNodServo;
    private static Servo axelServo;
    private static Servo topNodServo;
    private static Servo topGripServo;

    private static int servoSwitch = 0;
    private static DcMotor LSlide, RSlide;

    static protected HuskyLens huskyLens;

    static protected ColorSensor colorSensor;

    private static DoubleSwitchedServo pushServos;
    private static DoubleSwitchedServo twistingBottomServos;
    private static TripleSwitchedServo swingServos;
    private static DoubleSwitchedServo bottomNodServos;
    private static DoubleSwitchedServo axelServos;
    private static DoubleSwitchedServo topNodServos;
    private static DoubleSwitchedServo topGripServos;

    private static LinearSlide linearSlide;

    public static Macro collectSpecimen;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        LPushServo = hardwareMap.get(Servo.class, "LIP");
        RPushServo = hardwareMap.get(Servo.class, "RIP");

        twistingBottomServo = hardwareMap.get(Servo.class, "ITS");

        LSwingServo = hardwareMap.get(Servo.class, "LOFS");
        RSwingServo = hardwareMap.get(Servo.class, "ROFS");

        bottomNodServo = hardwareMap.get(Servo.class, "IRS");

        axelServo = hardwareMap.get(Servo.class, "IFS");

        topNodServo = hardwareMap.get(Servo.class, "ORS");

        topGripServo = hardwareMap.get(Servo.class, "OGS");

        LSlide = hardwareMap.get(DcMotor.class, "LVLS");
        RSlide = hardwareMap.get(DcMotor.class, "RVLS");

        pushServos = new DoubleSwitchedServo(LPushServo, RPushServo, 0, 1);
        twistingBottomServos = new DoubleSwitchedServo(twistingBottomServo, 0, .5);
        swingServos = new TripleSwitchedServo(LSwingServo, RSwingServo, 0, .2, .55) {
            // RAHH ANONYMOUS CLASSES (actually useful though)
            @Override
            public void setDirections() {
                servo1.setDirection(Servo.Direction.FORWARD);
                servo2.setDirection(Servo.Direction.FORWARD);
            }
        };
        bottomNodServos = new DoubleSwitchedServo(bottomNodServo, 0, .95);
        axelServos = new DoubleSwitchedServo(axelServo, .33, .88);
        topNodServos = new DoubleSwitchedServo(topNodServo, .85, 0);
        topGripServos = new DoubleSwitchedServo(topGripServo, .35, .63);

        linearSlide = new LinearSlide(LSlide, RSlide, 1);

        Macro collectSpecimen = () -> {
            topNodServos.secondaryPos();
            sleep(500);
            topGripServos.secondaryPos();
            sleep(500);
            topNodServos.primaryPos();
            sleep(500);
            swingServos.tertiaryPos();
        };

        //huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        //colorSensor = hardwareMap.get(ColorSensor.class, "colorsensor");
        //huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            moveWheels(gamepad1.left_stick_x, gamepad1.left_stick_y);

            if (gamepad1.right_stick_y > .3) {
                linearSlide.ascend();
            } else if (gamepad1.right_stick_y < -.3) {
                linearSlide.descend();
            } else {
                linearSlide.disablePower();
            }

            if (gamepad1.b && delay()) {
                swingServos.quickSwitch();
            } else if (gamepad1.a && delay()) {
                topGripServos.quickSwitch();
            } else if (gamepad1.x && delay()) {
                collectSpecimen.activate();
            } else if (gamepad1.y && delay()) {
                topNodServos.quickSwitch();
            }
            updatePhoneConsole();
        }
    }

    private boolean delay() {
        return System.currentTimeMillis() - time >= 250;
    }
    public void updatePhoneConsole() {
        telemetry.update();
    }
}