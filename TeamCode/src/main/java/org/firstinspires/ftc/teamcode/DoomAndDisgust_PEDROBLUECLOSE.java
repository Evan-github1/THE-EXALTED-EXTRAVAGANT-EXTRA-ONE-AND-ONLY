package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;
import static org.firstinspires.ftc.teamcode.AutoConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@Autonomous
public class DoomAndDisgust_PEDROBLUECLOSE extends Movable {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private static DcMotorEx intakeMotor, launcherMotor1, launcherMotor2,transferMotor;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static DoubleSwitchedServo fires;
    private static Servo fork;
    private static DoubleSwitchedServo forks;
    private Follower follower;
    private static int iterations;
    private static Limelight3A limelight;
    private static double motorPowerFar, motorPowerClose;
    private static double rpm, tps, targetRPM, P, FClose, FFar, currentTargetRPM;
    private static PIDFCoefficients pidfCoefficients;
    private static double pastError;
    private static boolean followerActive;

    private PathChain scorePreload, goToLeavePose;

    public void runOpMode() throws InterruptedException{


        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotorEx.class,"INT");
        transferMotor = hardwareMap.get(DcMotorEx.class, "INT2");
        lt1 = hardwareMap.get(Servo.class,"LT1");
        lt2 = hardwareMap.get(Servo.class,"LT2");
        fire = hardwareMap.get(Servo.class, "FIRE");
        fires = new DoubleSwitchedServo(fire,.8,.4);;
        launcherMotor1 = hardwareMap.get(DcMotorEx.class,"LAU1");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class,"LAU2");
        fork = hardwareMap.get(Servo.class,"FORK");
        follower = createFollower(hardwareMap);
        motorPowerClose = 2200;
        motorPowerFar = 3050; //from 4800
        followerActive = true;
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.pipelineSwitch(0);
        targetRPM = motorPowerFar;
        P = 50;
        FClose = 16.8;
        FFar = 15;
        pidfCoefficients = new PIDFCoefficients(P,0,0.005,FFar);
        actionTimer = new Timer();


        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        iterations = 0;
        launcherMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        launcherMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);


        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(DOOM_BLUE_CLOSE_START,DOOM_BLUE_CLOSE_SCORE))
                .setLinearHeadingInterpolation(DOOM_BLUE_CLOSE_START.getHeading(),DOOM_BLUE_CLOSE_SCORE.getHeading())
                .build();
        goToLeavePose = follower.pathBuilder()
                .addPath(new BezierLine(DOOM_BLUE_CLOSE_SCORE,DOOM_BLUE_CLOSE_LEAVE))
                .setLinearHeadingInterpolation(DOOM_BLUE_CLOSE_SCORE.getHeading(),DOOM_BLUE_CLOSE_LEAVE.getHeading())
                .build();


        follower.setStartingPose(DOOM_BLUE_CLOSE_START);

        pathState = 0;
        AutoConfig.isRed = false;

        waitForStart();
        launcherMotor1.setVelocity(motorPowerClose / 60 * 28);
        launcherMotor2.setVelocity(motorPowerClose / 60 * 28);
        lt1.setPosition(.25);
        lt2.setPosition(.25);
        intakeMotor.setPower(1);
        sleep(1000);
        boolean breaked = false;

        while (opModeIsActive()) {

            if(followerActive) {
                follower.update();
            }
            AutoConfig.lastAutoEndPose = follower.getPose();
            telemetry.addData("Elapsed time:",actionTimer.getElapsedTimeSeconds());
            telemetry.update();
            switch (pathState) {

                case 0:
                    follower.followPath(scorePreload);
                    pathState++;
                    actionTimer.resetTimer();
                    break;

                case 1:
                    if(!follower.isBusy()) {
                        if (actionTimer.getElapsedTimeSeconds() < 3) {
                            if (iterations == 0) {
                                pastError = 0;
                                LeBotsEyes(pastError, true);
                            } else {
                                pastError = LeBotsEyes(pastError, false);
                                LeBotsEyes(pastError, true);
                            }
                            iterations++;
                        } else {
                            FLW.setPower(0);
                            FRW.setPower(0);
                            BRW.setPower(0);
                            BLW.setPower(0);
                            followerActive = false;
                            iterations = 0;
                            transferMotor.setPower(1);
                            fires.secondaryPos();
                            sleep(1000);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            follower.followPath(goToLeavePose);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        AutoConfig.lastAutoEndPose = follower.getPose();
                        breaked = true;
                    }
                    actionTimer.resetTimer();
                    break;
            }
            if(breaked) break;
        }

    }

    private double LeBotsEyes(double pastError, boolean adjustMotor){
        double desiredX;
        telemetry.addData("D", true);
        if(detectTagSelective(limelight,telemetry) != null){
            LLResultTypes.FiducialResult yes = detectTagSelective(limelight,telemetry);
            desiredX = -2;
            double smoothCoeff = 0.25;
            telemetry.addData("Yes is not null",true);
            double tx = yes.getTargetXDegrees();
            double currentError = desiredX - tx;
            double smoothedError = smoothCoeff*currentError + (1-smoothCoeff)*pastError;
            smoothedError = smoothedError/25;
            telemetry.addData(""+currentError,smoothedError);
            if(adjustMotor) {
                FLW.setPower(-smoothedError);
                FRW.setPower(smoothedError);
                BRW.setPower(smoothedError);
                BLW.setPower(-smoothedError);
                return 0.0;
            }
            return smoothedError;
        }else{
            return 0;
        }

    }

    @Override
    public void green() {

    }

    @Override
    public void purple() {

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
}
