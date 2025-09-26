package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp
public class BallLauncher extends LinearOpMode {

    private static DcMotor launcherMotorL, launcherMotorR;
    private static boolean onSwitch;
    private static double speed;
    private static long time;

    @Override
    public void runOpMode() throws InterruptedException {
        launcherMotorL = hardwareMap.get(DcMotor.class, "a");
        launcherMotorR = hardwareMap.get(DcMotor.class, "b");

        MotorConfigurationType motorConfigurationType = launcherMotorL.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        launcherMotorL.setMotorType(motorConfigurationType);
        MotorConfigurationType motorConfigurationType2 = launcherMotorR.getMotorType().clone();
        motorConfigurationType2.setAchieveableMaxRPMFraction(1.0);
        launcherMotorR.setMotorType(motorConfigurationType2);

        launcherMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        onSwitch = false;
        speed = .5;
        time = System.currentTimeMillis();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.y && delay()) {
                if (onSwitch) {
                    launcherMotorL.setPower(0);
                    launcherMotorR.setPower(0);
                } else {
                    launcherMotorL.setPower(speed);
                    launcherMotorR.setPower(speed);
                }
                onSwitch = !onSwitch;
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_up && delay()) {
                speed += .05;
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && delay()) {
                speed -= .05;
                time = System.currentTimeMillis();
            }

            telemetry.addData("Speed", speed);
            telemetry.addData("RPM", launcherMotorR.getPower()); // this doesn't work.
            telemetry.addData("On?", onSwitch);
            telemetry.update();
        }

    }

    private boolean delay() {
        return System.currentTimeMillis() - time >= 250;
    }

}
