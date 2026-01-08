package org.firstinspires.ftc.teamcode.TestTeleOps;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@TeleOp
public class PIDFTuning extends OpMode {

    private static DcMotorEx outtakeMotor;
    private static final double HIGH_VELOCITY = 2300, LOW_VELOCITY = 1450;
    private static double currentTargetVelocity;
    private static double F = 0, P = 0;
    private static double[] stepSizes = {10, 1, .1, .01, .001, .0001};
    private static int stepIndex;

    @Override
    public void init() {
        currentTargetVelocity = HIGH_VELOCITY;
        stepIndex = 1;

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (currentTargetVelocity == HIGH_VELOCITY) {
                currentTargetVelocity = LOW_VELOCITY;
            } else {
                currentTargetVelocity = HIGH_VELOCITY;
            }
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeMotor.setVelocity(currentTargetVelocity);

        double currentVelocity = -outtakeMotor.getVelocity();
        double error = currentTargetVelocity - currentVelocity;

        telemetry.addData("Target Velocity", currentTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currentVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

        telemetry.update();
    }
}