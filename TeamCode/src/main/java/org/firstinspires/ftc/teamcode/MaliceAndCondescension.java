package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTag;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@TeleOp
public class MaliceAndCondescension extends Movable implements LimelightTag, LimelightColor {

    private static Limelight3A limelight;
    private static Servo swivelServo;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        swivelServo = hardwareMap.get(Servo.class, "swivelServo");

        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (gamepad1.a) { // doesn't work
                int detectedTag = tagDetection(limelight, telemetry);

                if (detectedTag == -1 && swivelServo.getPosition() < 1.0) {
                    double nextPos = swivelServo.getPosition() + 0.02;
                    swivelServo.setPosition(nextPos);

                    sleep(20);
                }

                telemetry.addData("Tag detected", detectedTag);
                telemetry.addData("Servo pos", swivelServo.getPosition());
                telemetry.update();
            } else if (gamepad1.b) { // reset swivel servo position
                swivelServo.setPosition(0);
                swivelServo.setDirection(Servo.Direction.FORWARD);
            } else if (gamepad1.x) { // reset swivel servo position
                swivelServo.setPosition(1);
            }

            updatePhoneConsole();
        }
    }

    public void updatePhoneConsole() {
        telemetry.update();
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
}