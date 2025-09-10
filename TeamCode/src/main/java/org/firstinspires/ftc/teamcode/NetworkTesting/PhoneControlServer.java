package org.firstinspires.ftc.teamcode.NetworkTesting;

/*
    use this with the DesktopControlClient class
    look at my doc for more info: https://docs.google.com/document/d/1bxNabZWSuYS4XG__-KeR-mj_PQDcAdDYHsHEZVNLnCc/edit?tab=t.0
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.*;
import java.net.*;
import java.util.*;

@TeleOp
public class PhoneControlServer extends LinearOpMode {
    private static ServerSocket serverSocket;
    private static Socket socket;
    private static DataOutputStream out;
    private static DataInputStream in;
    // after a bit of scouring, I think HardwareDevice is the interface that includes servos and motors
    private static final Map<String, HardwareDevice> components = new HashMap<>();
    private static String msg = "";
    private static double motorBasePower = .1, servoBaseD = .05;
    public void runOpMode() {

        waitForStart();

        try {
            // creates the server
            serverSocket = new ServerSocket(1234);

            socket = serverSocket.accept();

            out = new DataOutputStream(socket.getOutputStream());
            in = new DataInputStream(socket.getInputStream());

        } catch (IOException e) { throw new RuntimeException(e); }

        while (opModeIsActive()) {
            telemetry.addData("Connection is running. Client IP is", socket.getInetAddress());

            // reads input from the client
            String input;
            try {
                input = in.readUTF();
            } catch (IOException e) {  throw new RuntimeException(e); }

            String[] words = input.trim().split("\\s+");

            // initiate a new robot component (servo, motor)
            if (words[0].equalsIgnoreCase("init")) {
                try {
                    components.put(words[1], hardwareMap.get(getComponentClass(words[2]), words[1]));
                    msg += "Initiated component. Name: " + words[1] + ", Hardware Device: " + words[2];
                } catch (Exception e) {
                    msg += "Robot part configuration failed. Error message:\n" + e;
                }
            // checks all initiated components
            } else if (words[0].equalsIgnoreCase("view") && words[1].equalsIgnoreCase("init")) {
                for (String name : components.keySet()) {
                    msg += name + " - " + components.get(name);
                    msg += "\n";
                }
            // ends the program
            } else if (words[0].equalsIgnoreCase("term")) {
                try {
                    msg += "Terminating the program!";
                    socket.close();
                    serverSocket.close();
                    requestOpModeStop();
                } catch (IOException e) { throw new RuntimeException(e); }
            // if the message starts with the name of an initiated component
            } else if (checkComponents(words[0]) != null) {
                HardwareDevice component = checkComponents(words[0]);
                try {
                    // commands for servos
                    if (component instanceof Servo) { // instanceof is used after a bajillion years of neglect :DD
                        Servo servo = (Servo) component; // bruh
                        if (words[1].equalsIgnoreCase("inc")) {
                            // increase servo position
                            try {
                                servo.setPosition(servo.getPosition() + Double.parseDouble(words[2]));
                            } catch (NumberFormatException | IndexOutOfBoundsException e2) {
                                servo.setPosition(servo.getPosition() + servoBaseD);
                            }
                            msg += "Increasing servo position to " + servo.getPosition();
                        } else if (words[1].equalsIgnoreCase("dec")) {
                            // decrease servo position
                            try {
                                servo.setPosition(servo.getPosition() - Double.parseDouble(words[2]));
                            } catch (NumberFormatException | IndexOutOfBoundsException e2) {
                                servo.setPosition(servo.getPosition() - servoBaseD);
                            }
                            msg += "Decreasing servo position to " + servo.getPosition();
                            // move servo to a custom position
                        } else if (words[1].equalsIgnoreCase("set") && words[2].equalsIgnoreCase("pos")) {
                            servo.setPosition(Double.parseDouble(words[3]));
                            msg += "Setting servo position to " + servo.getPosition();
                            // change the direction of the servo
                        } else if (words[1].equalsIgnoreCase("change") && words[2].equalsIgnoreCase("direction")) {
                            if (servo.getDirection().equals(Servo.Direction.FORWARD)) {
                                servo.setDirection(Servo.Direction.REVERSE);
                            } else {
                                servo.setDirection(Servo.Direction.FORWARD);
                            }
                            msg += "Setting servo direction to " + servo.getDirection();
                            // increase/decrease servo increment/decrement base to something else
                        } else if (words[1].equalsIgnoreCase("set") && words[2].equalsIgnoreCase("base")) {
                            servoBaseD = Double.parseDouble(words[3]);
                            msg += "Setting servo base increment/decrement to " + servoBaseD;
                        }
                    // commands for motors
                    } else if (component instanceof DcMotor) {
                        DcMotor motor = (DcMotor) component; // bruh x2
                        // sets the motor power to a certain number
                        if (words[1].equalsIgnoreCase("set") && words[2].equalsIgnoreCase("power")) {
                            motorBasePower =  Double.parseDouble(words[3]);
                            msg += "Setting motor base power to " + motorBasePower;
                            // runs the motor
                        } else if (words[1].equalsIgnoreCase("run")) {
                            new Thread(() -> {
                                motor.setPower(motorBasePower);

                                try {
                                    Thread.sleep(Long.parseLong(words[3]));
                                    motor.setPower(0);
                                } catch (NumberFormatException | IndexOutOfBoundsException | InterruptedException _) {
                                    // swallow exception
                                }
                            }).start();
                            msg += "Running motor";
                            // stops the motor
                        } else if (words[1].equalsIgnoreCase("stop")) {
                            motor.setPower(0);
                            // changes the direction of the motor
                        } else if (words[1].equalsIgnoreCase("change") && words[2].equalsIgnoreCase("direction")) {
                            if (motor.getDirection().equals(DcMotor.Direction.FORWARD)) {
                                motor.setDirection(DcMotor.Direction.REVERSE);
                            } else {
                                motor.setDirection(DcMotor.Direction.FORWARD);
                            }
                            msg += "Setting motor direction to " + motor.getDirection();
                        }
                    }
                } catch (Exception e) {
                    msg += "Accessing robot parts failed. Error message:\n" + e;
                }
            }

            // always sends and resets the message to the client
            try {
                out.writeUTF(msg);
                out.flush();
            } catch (IOException e) { throw new RuntimeException(e); }
            finally {
                msg = "";
                telemetry.update();
            }
        }
    }

    // finds if a robot component is initiated
    private HardwareDevice checkComponents(String s) {
        for (String name : components.keySet()) {
            if (name.equalsIgnoreCase(s)) {
                return components.get(name);
            }
        }
        return null; // no components found
    }

    // returns a robot component based on string name
    private Class<? extends HardwareDevice> getComponentClass(String name) { // RAHH WILDCARD
        // only supports servos and motors so far
        if (name.equalsIgnoreCase("servo")) {
            return Servo.class;
        } else if (name.equalsIgnoreCase("motor") || name.equalsIgnoreCase("dcmotor")) {
            return DcMotor.class;
        } else {
            return null;
        }
    }

}
