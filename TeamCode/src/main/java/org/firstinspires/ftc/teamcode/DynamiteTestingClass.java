package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

import java.util.List;

@TeleOp
public class DynamiteTestingClass extends Movable implements LimelightTags {

    private static Servo swivelServo;
    private static DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        swivelServo = hardwareMap.get(Servo.class, "swivelServo");
        limelight.pipelineSwitch(0); // april tags
        limelight.start();
        intake = hardwareMap.get(DcMotor.class, "intake"); // placeholder

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            if (gamepad1.a) {
                if (detectTag(telemetry) == -1 && swivelServo.getPosition() < 1.0) {
                    double nextPos = swivelServo.getPosition() + 0.0005;
                    swivelServo.setPosition(nextPos);
                }
                telemetry.addData("Servo Position", swivelServo.getPosition());

            } else if (gamepad1.b) { // reset swivel servo position
                swivelServo.setPosition(0);
                swivelServo.setDirection(Servo.Direction.FORWARD);
            }
            else if (gamepad1.x) { // reset swivel servo position
                swivelServo.setPosition(1);
            }

            if (gamepad2.a) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

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


}