package org.firstinspires.ftc.teamcode.ServoTesting;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestTwoServoPos extends LinearOpMode {

    static private Servo servo;
    static private Servo servo2;
    static private String servoName;
    static private String servoName2;
    static private int currentLetterNum;
    static private boolean capsLock;

    static private double servoPos, servoPos2;
    static private double incValue;
    static private boolean servoConfirmed, servoConfirmed2;
    static private boolean onServo1;

    static private long delay;

    // TODO: READ "HOW_TO_USE" BEFORE USING!

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        reset();
        waitForStart();
        while(opModeIsActive()) {
            if (servoConfirmed && servoConfirmed2) {
                if (addButtonDelay()) {
                    if (gamepad2.dpad_left) {
                        // dec servo pos
                        if (onServo1) {
                            servoPos -= incValue;
                        } else {
                            servoPos2 -= incValue;
                        }
                        delay = System.currentTimeMillis();
                    } else if (gamepad2.dpad_right) {
                        // inc servo pos
                        if (onServo1) {
                            servoPos += incValue;
                        } else {
                            servoPos2 += incValue;
                        }
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
                    } else if (gamepad2.y) {
                        onServo1 = !onServo1;
                        delay = System.currentTimeMillis();
                    } else if (gamepad2.b) {
                        servoPos += incValue;
                        servoPos2 += incValue;
                        delay = System.currentTimeMillis();
                    } else if (gamepad2.a) {
                        servoPos -= incValue;
                        servoPos2 -= incValue;
                        delay = System.currentTimeMillis();
                    }
                }

                // stop pesky floating point error :(
                servoPos = (double) Math.round(servoPos * 100) / 100;
                servoPos2 = (double) Math.round(servoPos2 * 100) / 100;

                servo.setPosition(servoPos);
                servo2.setPosition(servoPos2);

                // arrows to make it obvious which servo you're on
                if (onServo1) {
                    telemetry.addData("Servo position", servoPos + " <<");
                    telemetry.addData("Servo 2 position", servoPos2);
                } else {
                    telemetry.addData("Servo position", servoPos);
                    telemetry.addData("Servo 2 position", servoPos2 + " <<");
                }
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
                            if (onServo1) {
                                servoName += letters.charAt(currentLetterNum);
                            } else {
                                servoName2 += letters.charAt(currentLetterNum);
                            }
                        } else {
                            if (onServo1) {
                                servoName += (letters.toUpperCase()).charAt(currentLetterNum);
                            } else {
                                servoName2 += (letters.toUpperCase()).charAt(currentLetterNum);
                            }
                        }
                        delay = System.currentTimeMillis();
                    } else if (gamepad1.a) {
                        // delete last character
                        try {
                            if (onServo1) {
                                servoName = servoName.substring(0, servoName.length() - 1);
                            } else {
                                servoName2 = servoName2.substring(0, servoName2.length() - 1);
                            }
                        } catch (Exception e) {} // swallow exception
                        delay = System.currentTimeMillis();
                    } else if (gamepad1.y) {
                        onServo1 = !onServo1;
                        delay = System.currentTimeMillis();
                    } else if (gamepad1.options) {
                        fetchServo();
                    }
                }
                if (onServo1) {
                    telemetry.addData("Current servo #1 name", servoName + " <<");
                    telemetry.addData("Current servo #2 name", servoName2);
                } else {
                    telemetry.addData("Current servo #1 name", servoName);
                    telemetry.addData("Current servo #2 name", servoName2 + " <<");
                }
                telemetry.addData("Current letter", letters.charAt(currentLetterNum));
                telemetry.addData("Caps Lock", capsLock);
            }

            telemetry.update();
        }
    }

    private void fetchServo() {
        // try to fetch servos with current servoName
        try {
            servo = hardwareMap.get(Servo.class, servoName);

            telemetry.addData("Success", servoName + " is detected.");
            servoConfirmed = true;
        } catch (Exception e) {
            telemetry.addData("Error", "\"" + servoName + "\"" +
                    " is not a servo. Please re-check your spelling and/or configuration.");
            servoConfirmed = false;
        }
        try {
            servo2 = hardwareMap.get(Servo.class, servoName2);
            telemetry.addData("Success", servoName2 + " is detected.");
            servoConfirmed2 = true;
        } catch (Exception e) {
            telemetry.addData("Error", "\"" + servoName2 + "\"" +
                    " is not a servo. Please re-check your spelling and/or configuration.");
            servoConfirmed2 = false;
        }
    }

    private void reset() {
        servoName = "";
        servoName2 = "";
        currentLetterNum = 0;
        capsLock = false;
        onServo1 = true;
        servoPos = 0;
        servoPos2 = 0;
        incValue = .05;
        servoConfirmed = false;
        servoConfirmed2 = false;
        delay = System.currentTimeMillis();
    }

    private boolean addButtonDelay() {
        return System.currentTimeMillis() - delay > 300;
    }
}