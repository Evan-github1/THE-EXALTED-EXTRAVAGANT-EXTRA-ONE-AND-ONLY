package org.firstinspires.ftc.teamcode.TestTeleOps;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;

@Disabled
@TeleOp
public class PIDFTuningArhans extends Movable implements LimelightTags {

    private static DcMotor intakeMotor;
    private static DcMotorEx launcherMotor1, launcherMotor2;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static TripleSwitchedServo fires;
    private static Servo fork;
    private static DoubleSwitchedServo forks;
    private static boolean loading;
    private static boolean shooting;
    private static Limelight3A limelight;
    private static Thread orientRobot;
    private static boolean tracking;
    private static double pastError;
    private static int iterations;
    private static double motorPowerFar, motorPowerClose;
    private static double rpm, tps, targetRPM, P, F, currentTargetRPM;
    private static double[] stepSizes;
    private static int stepIndex;
    private static PIDFCoefficients pidfCoefficients;




    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotor.class,"INT");
        lt1 = hardwareMap.get(Servo.class,"LT1");
        lt2 = hardwareMap.get(Servo.class,"LT2");
        fire = hardwareMap.get(Servo.class, "FIRE");fires = new TripleSwitchedServo(fire,.55,.49,.35);
        launcherMotor1 = hardwareMap.get(DcMotorEx.class,"LAU1");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class,"LAU2");
        fork = hardwareMap.get(Servo.class,"FORK");
        forks = new DoubleSwitchedServo(fork,.05,.68);
        loading = false;
        shooting = false;
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        motorPowerClose = 2500;
        motorPowerFar = 4500;
        targetRPM = motorPowerFar;
        stepSizes = new double[]{10, 1, 0.1, 0.01, 0.001,0.0001};
        stepIndex = 1;
        pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        launcherMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        launcherMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pastError = 0;
        iterations = 0;

        enableEncoders();

        waitForStart();

        fires.primaryPos();
        forks.primaryPos();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            tps = launcherMotor2.getVelocity();

            rpm = tps * 60 / 28;
            telemetry.addData("RPM",rpm);
            telemetry.addData("Target RPM",targetRPM);

            if(gamepad1.leftBumperWasPressed()){
                stepIndex = (stepIndex - 1) % stepSizes.length;
            }else if(gamepad1.rightBumperWasPressed()){
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            if(gamepad1.dpadDownWasPressed()){
                F -= stepSizes[stepIndex];
                pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                launcherMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            }else if(gamepad1.dpadUpWasPressed()){
                F += stepSizes[stepIndex];
                pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                launcherMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            }else if(gamepad1.dpadLeftWasPressed()){
                P -= stepSizes[stepIndex];
                pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                launcherMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            }else if(gamepad1.dpadRightWasPressed()){
                P += stepSizes[stepIndex];
                pidfCoefficients = new PIDFCoefficients(P,0,0,F);
                launcherMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
            }

            double error = targetRPM - rpm;

            if (shooting) {
                launcherMotor1.setVelocity(targetRPM / 60 * 28);
                launcherMotor2.setVelocity(targetRPM / 60 * 28);
            }else{
                launcherMotor1.setVelocity(0);
                launcherMotor2.setVelocity(0);
            }
            telemetry.addData("Target Velocity", targetRPM);
            telemetry.addData("Current Velocity", "%.2f", rpm);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addLine("------------------------------");
            telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
            telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
            telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
            telemetry.update();
        }
    }



    @Override
    public void tag20() {

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
    public void tag24() {

    }

    @Override
    public void nothing() {

    }
}
