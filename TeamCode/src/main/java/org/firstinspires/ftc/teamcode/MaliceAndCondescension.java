package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

import java.util.List;

@TeleOp
public class MaliceAndCondescension extends Movable implements LimelightColor { // robot #22335

    private static Limelight3A limelight;
    private static Servo swivelServo;
    private static List<LLResultTypes.FiducialResult> results;
    private static DcMotor intakeMotor;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways = new DoubleSwitchedServo(gatewayServo, .23, .76);

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        swivelServo = hardwareMap.get(Servo.class, "swivelServo");
        limelight.pipelineSwitch(0); // april tags
        limelight.start();
        intakeMotor = hardwareMap.get(DcMotor.class, "intake"); // placeholder
        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            if (gamepad1.a) {
                LLResult result = limelight.getLatestResult();
                if (result.isValid()) {
                    results = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult r : results) {
                        int id = r.getFiducialId();
                        switch (id) {
                            case 21:
                                tag21();
                                break; // GPP
                            case 22:
                                tag22();
                                break; // PGP
                            case 23:
                                tag23();
                                break; // PPG
                        }
                        telemetry.addData("AprilTag ID", id);
                    }
                } else if (swivelServo.getPosition() < 1.0) {
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
                intakeMotor.setPower(1);
            } else if (gamepad2.b && delay()) {
                gateways.quickSwitch();
            } else {
                intakeMotor.setPower(0);
            }

            
            telemetry.update();
        }
    }

    public void tag21() {

    }

    public void tag22() {

    }

    public void tag23() {

    }

    @Override
    public void green() {

    }

    @Override
    public void purple() {

    }


}