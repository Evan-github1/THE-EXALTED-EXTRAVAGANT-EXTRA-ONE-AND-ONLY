package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;

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
public class DoomAndDisgust_PEDROBLUE extends Movable {

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


    private final Pose startPose = new Pose(144-96, 0 + robotLength()/2, Math.PI-Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(144-88.6,11.7,Math.PI-Math.toRadians(-115.5));
    private final Pose ball1PickupStart = new Pose(144-100,36,Math.PI-0);
    private final Pose ball1PickupEnd = new Pose(144-137,36,Math.PI-0);
    private final Pose ball2PickupStart = new Pose(144-100,60,Math.PI-0);
    private final Pose ball2PickupEnd = new Pose(144-137,60,Math.PI-0);
    private final Pose ball3PickupStart = new Pose(144-100,84,Math.PI-0);
    private final Pose ball3PickupEnd = new Pose(144-130,84,Math.PI-0);

    private PathChain scorePreload, goToPickup1, grabPickup1, scorePickup1, goToPickup2, grabPickup2, scorePickup2, goToPickup3, grabPickup3, scorePickup3;

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
        motorPowerClose = 2500;
        motorPowerFar = 3900; //from 4800
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
                .addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();
        goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, ball1PickupStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), ball1PickupStart.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(ball1PickupStart,ball1PickupEnd))
                .setConstantHeadingInterpolation(ball1PickupStart.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(ball1PickupEnd, scorePose))
                .setLinearHeadingInterpolation(ball1PickupEnd.getHeading(), scorePose.getHeading())
                .build();

        goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,ball2PickupStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),ball2PickupStart.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(ball2PickupStart, ball2PickupEnd))
                .setLinearHeadingInterpolation(ball2PickupStart.getHeading(), ball2PickupEnd.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(ball2PickupEnd, scorePose))
                .setLinearHeadingInterpolation(ball2PickupEnd.getHeading(), scorePose.getHeading())
                .build();

        goToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, ball3PickupStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), ball3PickupStart.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(ball3PickupStart,ball3PickupEnd))
                .setConstantHeadingInterpolation(ball3PickupEnd.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(ball3PickupEnd, ball3PickupStart, scorePose))
                .setLinearHeadingInterpolation(ball3PickupEnd.getHeading(), scorePose.getHeading())
                .build();



        follower.setStartingPose(startPose);

        pathState = 0;

        waitForStart();
        launcherMotor1.setVelocity(motorPowerFar);
        launcherMotor2.setVelocity(motorPowerFar);
        lt1.setPosition(.4375); //from 0.45 for both
        lt2.setPosition(.4375);
        intakeMotor.setPower(1);
        sleep(1000);
        boolean breaked = false;

        while (opModeIsActive()) {

            if(followerActive) {
                follower.update();
            }
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
                            follower.followPath(goToPickup1);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup1,0.75,true);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup1);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 3) {
                            if (iterations == 0) {
                                pastError = 0;
                                LeBotsEyes(pastError, true);
                            } else {
                                pastError = LeBotsEyes(pastError, false);
                                LeBotsEyes(pastError, true);
                            }
                            iterations++;
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
                        }
                    }
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup2,.75,true);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup2);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 3) {
                            if (iterations == 0) {
                                pastError = 0;
                                LeBotsEyes(pastError, true);
                            } else {
                                pastError = LeBotsEyes(pastError, false);
                                LeBotsEyes(pastError, true);
                            }
                            iterations++;
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
                            follower.followPath(goToPickup3);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;
                case 8:
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup3,.75,true);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 9:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup3);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 10:
                    if(!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 3){
                            if(iterations == 0 ){
                                pastError = 0;
                                LeBotsEyes(pastError,true);
                            }else{
                                pastError = LeBotsEyes(pastError,false);
                                LeBotsEyes(pastError,true);
                            }
                            iterations++;
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
                        }
                    }
                    break;
                case 11:
                    if(!follower.isBusy()) breaked = true;
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
