package org.firstinspires.ftc.teamcode.TestTeleOps;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;


@TeleOp
public class PIDFTuning extends OpMode {

    private static DcMotorEx outtakeMotor;
    private static final double HIGH_VELOCITY = 2000, LOW_VELOCITY = 1500;
    private static double currentTargetVelocity;
    private static double F = 0, P = 0;
    private static double[] stepSizes = {10, 1, .1, .01, .001, .0001};
    private static int stepIndex;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;

    @Override
    public void init() {
        currentTargetVelocity = HIGH_VELOCITY;
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
        outtakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        outtakeMotor.setVelocity(currentTargetVelocity);

        double currentVelocity = -outtakeMotor.getVelocity();
        double error = currentTargetVelocity - currentVelocity;

        if (gamepad1.left_trigger >= .5 && delay(1001)) {
            liftRightWiper();
            time = System.currentTimeMillis();
        } else if (gamepad1.right_trigger >= .5 && delay(1001)) {
            liftLeftWiper();
            time = System.currentTimeMillis();
        }

        telemetry.addData("Target Velocity", currentTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currentVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

        telemetry.update();
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
            try {
                sleep(250);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            wipersR.primaryPos();
        }).start();
    }

    private void liftLeftWiper() {
        new Thread(() -> {
            wipersL.secondaryPos();
            try {
                sleep(250);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            wipersL.primaryPos();
        }).start();
    }
}