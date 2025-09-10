package org.firstinspires.ftc.teamcode.ServoTesting;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

// TODO: READ "HOW_TO_USE" BEFORE USING!

public class TestServoPos extends LinearOpMode {

    static private Servo servo;
    static private String servoName;
    static private int currentLetterNum;
    static private boolean capsLock;

    static private double servoPos;
    static private double incValue;
    static private boolean servoConfirmed;

    static private long delay;

    public void runOpMode() {
        telemetry.addData("Status",  "Initialized");
        telemetry.update();

        reset();
        waitForStart();

        while(opModeIsActive()) {
            if (servoConfirmed) {
                if (addButtonDelay()) {
                    if (gamepad2.dpad_left) {
                        // inc servo pos
                        servoPos -= incValue;
                        delay = System.currentTimeMillis();
                    } else if (gamepad2.dpad_right) {
                        // dec servo pos
                        servoPos += incValue;
                        delay = System.currentTimeMillis();
                    } else if (gamepad2.dpad_up) {
                        // inc inc value
                        incValue += .02;
                        delay = System.currentTimeMillis();
                    } else if (gamepad2.dpad_down) {
                        // dec inc value
                        incValue -= .02;
                        delay = System.currentTimeMillis();
                    } else if (gamepad2.right_bumper && gamepad2.left_bumper) {
                        reset();
                    }
                }
                servo.setPosition(servoPos);
                telemetry.addData("Servo position", servoPos);
                telemetry.addData("Increment value", incValue);
            } else {
                final String letters = "abcdefghijklmnopqrstuvwxyz";
                if (addButtonDelay()) {
                    if (gamepad1.dpad_right && currentLetterNum < 25) {
                        // go to next letter
                        currentLetterNum++;
                        delay = System.currentTimeMillis();
                    } else if (gamepad1.dpad_left && currentLetterNum > 0) {
                        // go to previous letter
                        currentLetterNum--;
                        delay = System.currentTimeMillis();
                    } else if (gamepad1.dpad_up) {
                        capsLock = true;
                    } else if (gamepad1.dpad_down) {
                        capsLock = false;
                    } else if (gamepad1.b) {
                        // add character to servo name
                        if (!capsLock) {
                            servoName += letters.charAt(currentLetterNum);
                        } else {
                            servoName += (letters.toUpperCase()).charAt(currentLetterNum);
                        }
                        delay = System.currentTimeMillis();
                    } else if (gamepad1.a) {
                        // delete last character
                        try {
                            servoName = servoName.substring(0, servoName.length() - 1);
                        } catch (Exception e) {} // swallow exception
                        delay = System.currentTimeMillis();
                    } else if (gamepad1.options) {
                        fetchServo();
                    }
                }
                telemetry.addData("Current servo name", servoName);
                telemetry.addData("Current letter", letters.charAt(currentLetterNum));
                telemetry.addData("Caps Lock", capsLock);
            }

            telemetry.update();
        }
    }

    private void fetchServo() {
        // try to fetch servo with current servoName
        try {
            servo = hardwareMap.get(Servo.class, servoName);
            telemetry.addData("Success", servoName + " is detected.");
            servoConfirmed = true;
        } catch (Exception e) {
            telemetry.addData("Error", "\"" + servoName + "\"" +
                    " is not a servo. Please re-check your spelling and/or configuration.");
            servoConfirmed = false;
        }
    }

    private void reset() {
        servoName = "";
        currentLetterNum = 0;
        capsLock = false;

        servoPos = 0;
        incValue = .05;
        servoConfirmed = false;
        delay = System.currentTimeMillis();
    }

    private boolean addButtonDelay() {
        return System.currentTimeMillis() - delay > 300;
    }
}