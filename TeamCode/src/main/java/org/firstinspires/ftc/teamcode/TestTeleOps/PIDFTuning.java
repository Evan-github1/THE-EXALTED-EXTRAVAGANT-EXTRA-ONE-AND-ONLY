package org.firstinspires.ftc.teamcode.TestTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;


@TeleOp
public class PIDFTuning extends Movable {

    private static DcMotorEx outtakeMotor;
    private static final double HIGH_VELOCITY = 4050, LOW_VELOCITY = 2600;
    private static double rpm, tps, P, F, currentTargetRPM;
    private static double[] stepSizes = {10, 1, .1, .01, .001, .0001};
    private static int stepIndex;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;
    private static final double TPR = 28;

    @Override
    public void runOpMode() {
        currentTargetRPM = HIGH_VELOCITY;
        stepIndex = 1;
        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");
        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        outtakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        waitForStart();

        while (opModeIsActive()) {
            //F:
            //P:
            telemetry.addData("Status", "Running");

            if (gamepad1.yWasPressed()) {
                if (currentTargetRPM == HIGH_VELOCITY) {
                    currentTargetRPM = LOW_VELOCITY;
                } else {
                    currentTargetRPM = HIGH_VELOCITY;
                }
            }
            if (gamepad1.left_trigger >= .5 && delay(1001)) {
                liftRightWiper();
                time = System.currentTimeMillis();
            } else if (gamepad1.right_trigger >= .5 && delay(1001)) {
                liftLeftWiper();
                time = System.currentTimeMillis();
            }

            tps = outtakeMotor.getVelocity();

            rpm = (tps * 60) / TPR;
            telemetry.addData("RPM",rpm);
            telemetry.addData("Target RPM", currentTargetRPM);

            if(gamepad1.leftBumperWasPressed()){
                stepIndex = (stepIndex - 1) % stepSizes.length;
            }else if(gamepad1.rightBumperWasPressed()){
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            if(gamepad1.dpadDownWasPressed()){
                F -= stepSizes[stepIndex];
                pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            }else if(gamepad1.dpadUpWasPressed()){
                F += stepSizes[stepIndex];
                pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            }else if(gamepad1.dpadLeftWasPressed()){
                P -= stepSizes[stepIndex];
                pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            }else if(gamepad1.dpadRightWasPressed()){
                P += stepSizes[stepIndex];
                pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            }

            double error = currentTargetRPM + rpm;

            outtakeMotor.setVelocity((currentTargetRPM / 60) * TPR);

            telemetry.addData("Target Velocity", currentTargetRPM);
            telemetry.addData("Current Velocity", "%.2f", rpm);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addLine("------------------------------");
            telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
            telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
            telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
            telemetry.update();
        }
    }



    public boolean delay() {
        return System.currentTimeMillis() >= time + 250;
    }

    public boolean delay(long duration) {
        return System.currentTimeMillis() >= time + duration;
    }


    private void liftRightWiper() {
        new Thread(() -> {
            wipersR.secondaryPos();
            sleep(250);
            wipersR.primaryPos();
        }).start();
    }

    private void liftLeftWiper() {
        new Thread(() -> {
            wipersL.secondaryPos();
            sleep(250);
            wipersL.primaryPos();
        }).start();
    }
}