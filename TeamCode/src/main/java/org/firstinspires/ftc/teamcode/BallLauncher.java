package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp
@Disabled
public class BallLauncher extends LinearOpMode {

    private static DcMotor launcherMotorL, launcherMotorR;
    private static boolean onSwitch;
    private static double speed;
    private static long time;
    public static final int TICKS_PER_REV = 28;
    public static int targetRPM = 3000;
    public static int localStartPos,lastPos;
    public static long localTimeMillis,lastTime;
    public static double RPM;
    public static boolean started = false;

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


        launcherMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        onSwitch = false;
        speed = .1;
        time = System.currentTimeMillis();

        waitForStart();



        while (opModeIsActive()) {
            if (!started) {
                launcherMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                launcherMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                started = true;
            }
            localStartPos = launcherMotorL.getCurrentPosition() - lastPos;
            localTimeMillis = System.currentTimeMillis()-lastTime;
            if (gamepad1.y && delay()) {
                onSwitch = !onSwitch;
                time = System.currentTimeMillis();
                if(onSwitch){
                    //globalStartPos = launcherMotorL.getCurrentPosition();
                    //globalTimeMillis = System.currentTimeMillis();

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

            if (localTimeMillis != 0) RPM = ((double) localStartPos / TICKS_PER_REV) * 60000 / (localTimeMillis);


            if (onSwitch) {
                launcherMotorL.setPower(speed);
                launcherMotorR.setPower(speed);

            } else {
                launcherMotorL.setPower(0);
                launcherMotorR.setPower(0);
            }

            lastTime = System.currentTimeMillis();
            lastPos = launcherMotorL.getCurrentPosition();

            telemetry.addData("Local pos",localStartPos);
            telemetry.addData("localTimeMillis",localTimeMillis);
            telemetry.addData("Current position",launcherMotorR.getCurrentPosition());
            telemetry.addData("Speed", speed);
            telemetry.addData("Actual RPM",RPM);
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
