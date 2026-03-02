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
public class DoomAndDisgust_PEDROREDFAR extends Movable {

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private static DcMotorEx intakeMotor, launcherMotor1, launcherMotor2, transferMotor;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static DoubleSwitchedServo fires;
    private static DoubleSwitchedServo forks;
    private Follower follower;
    private static int iterations;
    private static Limelight3A limelight;
    private static double motorPowerFar, motorPowerClose;
    private static double rpm, tps, targetRPM, P, FClose, FFar, currentTargetRPM;
    private static PIDFCoefficients pidfCoefficients;
    private static double pastError;
    private static boolean followerActive;



    private PathChain scorePreload, goToPickup1, grabPickup1, scorePickup1, goToPickup2, grabPickup2, scorePickup2, goToPickup3, grabPickup3, scorePickup3, clearClassifier1, clearClassifier2;
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
        follower = createFollower(hardwareMap);
        motorPowerClose = 2500;
        motorPowerFar = 3750; //from 4800
        followerActive = true;
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.pipelineSwitch(0);
        targetRPM = motorPowerFar;
        P = 50;
        FClose = 16.8;
        FFar = 15.8;
        pidfCoefficients = new PIDFCoefficients(P,0,0,FFar);
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

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_START,RED_FAR_SCORE))
                .setLinearHeadingInterpolation(RED_FAR_START.getHeading(),RED_FAR_SCORE.getHeading())
                .build();
        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_SCORE, RED_BALL1_START))
                .setLinearHeadingInterpolation(RED_FAR_SCORE.getHeading(), RED_BALL1_START.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL1_START,RED_BALL1_END))
                .setConstantHeadingInterpolation(RED_BALL1_START.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL1_END, RED_FAR_SCORE))
                .setLinearHeadingInterpolation(RED_BALL1_END.getHeading(), RED_FAR_SCORE.getHeading())
                .build();

        goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RED_CLOSE_SCORE,RED_BALL2_START))
                .setLinearHeadingInterpolation(RED_CLOSE_SCORE.getHeading(),RED_BALL2_START.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL2_START, RED_BALL2_END))
                .setLinearHeadingInterpolation(RED_BALL2_START.getHeading(), RED_BALL2_END.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(RED_BALL2_END, RED_BALL2_START,RED_FAR_SCORE))
                .setLinearHeadingInterpolation(RED_BALL2_END.getHeading(), RED_FAR_SCORE.getHeading())
                .build();

        goToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_SCORE, RED_BALL3_START))
                .setLinearHeadingInterpolation(RED_FAR_SCORE.getHeading(), RED_BALL3_START.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL3_START,RED_BALL3_END))
                .setConstantHeadingInterpolation(RED_BALL3_END.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_CLEAR, RED_CLOSE_SCORE))
                .setLinearHeadingInterpolation(RED_BALL3_END.getHeading(),RED_FAR_SCORE.getHeading())
                .build();

        clearClassifier1 = follower.pathBuilder()
                .addPath(new BezierCurve(RED_BALL3_END,RED_BALL3_START,DOOM_RED_FAR_CLEAR_READY))
                .setConstantHeadingInterpolation(0)
                .build();

        clearClassifier2 = follower.pathBuilder()
                .addPath(new BezierLine(DOOM_RED_FAR_CLEAR_READY,RED_FAR_CLEAR))
                .setConstantHeadingInterpolation(0)
                .build();


        follower.setStartingPose(RED_FAR_START);

        pathState = 0;
        AutoConfig.isRed = true;

        waitForStart();
        launcherMotor1.setVelocity(motorPowerFar / 60 * 28);
        launcherMotor2.setVelocity(motorPowerFar / 60 * 28);
        lt1.setPosition(.37);
        lt2.setPosition(.37); //.37 for far, .25 for close
        intakeMotor.setPower(1);
        sleep(500);
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
                        if (actionTimer.getElapsedTimeSeconds() < 2.4) {
                            try {
                                if (iterations == 0) {
                                    pastError = 0;
                                    LeBotsEyes(pastError, true);
                                } else {
                                    pastError = LeBotsEyes(pastError, false);
                                    LeBotsEyes(pastError, true);
                                }
                                iterations++;
                            }catch(Exception ignored){};
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
                            follower.followPath(goToPickup3);
                            pathState++;
                            actionTimer.resetTimer();
                            lt1.setPosition(.25);
                            lt2.setPosition(.25);
                            launcherMotor1.setVelocity(motorPowerClose/60*28);
                            launcherMotor2.setVelocity(motorPowerClose/60*28);
                        }
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup3,1,true);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 3:
                    if(!follower.isBusy()){
                        follower.followPath(clearClassifier1);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;

                case 4:
                    if(!follower.isBusy()){
                        follower.followPath(clearClassifier2);
                        pathState++;
                    }
                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup3);
                        pathState+=2;
                    }
                    actionTimer.resetTimer();
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 2.4) {
                            try {
                                if (iterations == 0) {
                                    pastError = 0;
                                    LeBotsEyes(pastError, true);
                                } else {
                                    pastError = LeBotsEyes(pastError, false);
                                    LeBotsEyes(pastError, true);
                                }
                                iterations++;
                            }catch(Exception ignored){};
                        }else {
                            FLW.setPower(0);
                            FRW.setPower(0);
                            BRW.setPower(0);
                            BLW.setPower(0);
                            followerActive = false;
                            iterations = 0;
                            transferMotor.setPower(1);
                            fires.secondaryPos();
                            disablePower();
                            sleep(1000);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            follower.followPath(goToPickup2);
                            pathState++;
                            actionTimer.resetTimer();
                            lt1.setPosition(.37);
                            lt2.setPosition(.37);
                            launcherMotor1.setVelocity(motorPowerFar/60*28);
                            launcherMotor2.setVelocity(motorPowerFar/60*28);
                        }
                    }
                    break;
                case 8:
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup2,1,true);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 9:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup2);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 10:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 2.4) {
                            try {
                                if (iterations == 0) {
                                    pastError = 0;
                                    LeBotsEyes(pastError, true);
                                } else {
                                    pastError = LeBotsEyes(pastError, false);
                                    LeBotsEyes(pastError, true);
                                }
                                iterations++;
                            }catch(Exception ignored){};
                        }else {
                            FLW.setPower(0);
                            FRW.setPower(0);
                            BRW.setPower(0);
                            BLW.setPower(0);
                            followerActive = false;
                            iterations = 0;
                            transferMotor.setPower(1);
                            fires.secondaryPos();
                            disablePower();
                            sleep(1000);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            follower.followPath(goToPickup1);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;
                case 11:
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup1,1,true);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 12:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup1);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 13:
                    if(!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 2.4){
                            try {
                                if (iterations == 0) {
                                    pastError = 0;
                                    LeBotsEyes(pastError, true);
                                } else {
                                    pastError = LeBotsEyes(pastError, false);
                                    LeBotsEyes(pastError, true);
                                }
                                iterations++;
                            }catch(Exception ignored){};
                        }else {
                            FLW.setPower(0);
                            FRW.setPower(0);
                            BRW.setPower(0);
                            BLW.setPower(0);
                            followerActive = false;
                            iterations = 0;
                            transferMotor.setPower(1);
                            fires.secondaryPos();
                            disablePower();
                            sleep(1000);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            follower.followPath(goToPickup1);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;
                case 14:
                    if(!follower.isBusy()) {
                        AutoConfig.isRed = true;
                        AutoConfig.lastAutoEndPose = follower.getPose();
                        breaked = true;
                    }
            }
            if(breaked) break;
        }


    }

    private double LeBotsEyes(double pastError, boolean adjustMotor){
        double desiredX;
        telemetry.addData("D", true);
        if(detectTagSelective(limelight,telemetry) != null){
            LLResultTypes.FiducialResult yes = detectTagSelective(limelight,telemetry);
            desiredX = 0;
            double smoothCoeff = 0.6;
            telemetry.addData("Yes is not null",true);
            double tx = yes.getTargetXDegrees();
            double currentError = desiredX - tx;
            double smoothedError = smoothCoeff*currentError + (1-smoothCoeff)*pastError;
            smoothedError = smoothedError/15;
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
