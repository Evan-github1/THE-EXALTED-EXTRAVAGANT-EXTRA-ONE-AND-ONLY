package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTag;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

import java.time.Instant;

@TeleOp
public class MaliceAndCondescension extends Movable implements LimelightTag, LimelightColor {

    private static Limelight3A limelight;
    private static Servo swivelServo;
    private int detectedTagID;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        swivelServo = hardwareMap.get(Servo.class, "swivelServo");
        detectedTagID = -1;
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Apple", true);
            telemetry.addData("Banana", true);
            telemetry.addData("Status", "Running");
            if (gamepad1.a) {
                tagDetection(limelight, telemetry);
                if (detectedTagID == -1 && swivelServo.getPosition() < 1.0) {
                    double nextPos = swivelServo.getPosition() + 0.02;
                    swivelServo.setPosition(nextPos);
                }

                telemetry.addData("Tag detected", detectedTagID);
                telemetry.addData("Servo pos", swivelServo.getPosition());
            } else if (gamepad1.b) { // reset swivel servo position
                swivelServo.setPosition(0);
                swivelServo.setDirection(Servo.Direction.FORWARD);
            }
            else if (gamepad1.x) { // reset swivel servo position
                swivelServo.setPosition(1);
            }
            telemetry.addData("Carrot", true);
            telemetry.addData("Date",true);









            telemetry.update();
        }
    }

    @Override
    public void tag21() {
        detectedTagID = 21;
    }

    @Override
    public void tag22() {
        detectedTagID = 22;
    }

    @Override
    public void tag23() {
        detectedTagID = 23;
    }

    @Override
    public void nothingDetected() {
        detectedTagID = -1;
    }

    @Override
    public void green() {

    }

    @Override
    public void purple() {

    }


}