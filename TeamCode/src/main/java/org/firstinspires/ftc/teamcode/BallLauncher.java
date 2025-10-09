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
    public static final int TICKS_PER_REV = 28;
    public static int targetRPM = 3000;
    public static int localStartPos,globalStartPos;
    public static long localTimeMillis,globalTimeMillis;
    public static double RPM;

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

        launcherMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcherMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        onSwitch = false;
        speed = .1;
        time = System.currentTimeMillis();

        waitForStart();



        while (opModeIsActive()) {
            if (gamepad1.y && delay()) {
                onSwitch = !onSwitch;
                time = System.currentTimeMillis();
                if(!onSwitch){
                    globalStartPos = launcherMotorL.getCurrentPosition();
                    globalTimeMillis = System.currentTimeMillis();
                }
            } else if (gamepad1.dpad_up && delay()) {
                speed += .05;
                targetRPM += 100;
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && delay()) {
                speed -= .05;
                targetRPM -= 100;
                time = System.currentTimeMillis();
            }

            try {
                RPM = (localStartPos / TICKS_PER_REV) * 60000 / localTimeMillis;
            }catch(Exception ignored){

            }

            if (onSwitch) {
                launcherMotorL.setPower(speed);
                launcherMotorR.setPower(speed);
                localStartPos = launcherMotorL.getCurrentPosition() - globalStartPos;
                localTimeMillis = System.currentTimeMillis()-globalTimeMillis;
            } else {
                launcherMotorL.setPower(0);
                launcherMotorR.setPower(0);

            }

            telemetry.addData("Local pos",localStartPos);
            telemetry.addData("Speed", speed);
            telemetry.addData("RPM", launcherMotorR.getPower());
            telemetry.addData("Actual RPM",RPM);
            telemetry.addData("On?", onSwitch);
            telemetry.update();
        }

    }

    private boolean delay() {
        return System.currentTimeMillis() - time >= 250;
    }
    private boolean delay(long cooldown) {
        return System.currentTimeMillis() - time >= cooldown;
    }
}
