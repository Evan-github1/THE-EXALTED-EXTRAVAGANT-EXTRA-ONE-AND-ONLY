package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotWidth;

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
public class UhUhUhUhUhUhBLUEFAR extends Movable {

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


    private final Pose startPose = new Pose(141.5-96, 0 + robotLength()/2, Math.PI-Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(141.5-88.6,11.7,Math.PI-Math.toRadians(-115.5)); //TODO: UPDATE THIS
    private final Pose ball1PickupStart = new Pose(141.5-95,36,Math.PI-0);
    private final Pose ball1PickupEnd = new Pose(141.5-130,36,Math.PI-0);
    private final Pose ball2PickupStart = new Pose(141.5-95,60,Math.PI-0);
    private final Pose ball2PickupEnd = new Pose(141.5-130,60,Math.PI-0);
    private final Pose ball3PickupStart = new Pose(141.5-95,84,Math.PI-0);
    private final Pose ball3PickupEnd = new Pose(141.5-125,84,Math.PI-0);
    private final Pose clearClassifier1 = new Pose(141.5-125,72,Math.PI-0);
    private final Pose clearClassifier2 = new Pose(141.5-115,72,Math.PI-0);
    private final Pose pickUpCorner1 = new Pose(robotWidth()/2,30+robotLength()/2,Math.PI-Math.toRadians(-90));
    private final Pose pickUpCorner2 = new Pose(robotWidth()/2,robotLength()/2+3,Math.PI-Math.toRadians(-90));

    private final Pose closeShoot = new Pose(141.5-85,85,-Math.PI/4);


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
        motorPowerClose = 2300;
        motorPowerFar = 3600; //from 4800
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
        launcherMotor1.setPIDFCoefficients(RUN_USING_ENCODER,pidfCoefficients);
        launcherMotor2.setPIDFCoefficients(RUN_USING_ENCODER,pidfCoefficients);

        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose,scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();

        PathChain goToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,ball3PickupStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),ball3PickupStart.getHeading())
                .build();

        PathChain grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(ball3PickupStart,ball3PickupEnd))
                .setLinearHeadingInterpolation(ball3PickupStart.getHeading(),ball3PickupEnd.getHeading())
                .build();

        PathChain scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(clearClassifier1,closeShoot))
                .setLinearHeadingInterpolation(clearClassifier1.getHeading(),closeShoot.getHeading())
                .build();

        PathChain clearClassifier = follower.pathBuilder()
                .addPath(new BezierCurve(ball3PickupEnd, clearClassifier2, clearClassifier1))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        PathChain goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,ball2PickupStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),ball2PickupStart.getHeading())
                .build();

        PathChain grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(ball2PickupStart,ball2PickupEnd))
                .setLinearHeadingInterpolation(ball2PickupStart.getHeading(),ball2PickupEnd.getHeading())
                .build();

        PathChain scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(ball2PickupEnd,closeShoot))
                .setLinearHeadingInterpolation(ball2PickupEnd.getHeading(),scorePose.getHeading())
                .build();

        PathChain goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,ball1PickupStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(),ball1PickupStart.getHeading())
                .build();

        PathChain grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(ball1PickupStart,ball1PickupEnd))
                .setLinearHeadingInterpolation(ball1PickupStart.getHeading(),ball1PickupEnd.getHeading())
                .build();

        PathChain scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(ball1PickupEnd,scorePose))
                .setLinearHeadingInterpolation(ball1PickupEnd.getHeading(),scorePose.getHeading())
                .build();

        PathChain getCorner = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickUpCorner1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpCorner1.getHeading())
                .build();

        PathChain scoreCorner1 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpCorner2,scorePose))
                .setLinearHeadingInterpolation(pickUpCorner2.getHeading(), scorePose.getHeading())
                .build();

        PathChain getCorner2 = follower.pathBuilder()
                .addPath(new BezierLine(pickUpCorner1,pickUpCorner2))
                .setConstantHeadingInterpolation(pickUpCorner1.getHeading())
                .build();

        follower.setStartingPose(startPose);

        pathState = 0;

        waitForStart();
        launcherMotor1.setVelocity(motorPowerFar / 60 * 28);
        launcherMotor2.setVelocity(motorPowerFar / 60 * 28);
        launcherMotor1.setPIDFCoefficients(RUN_USING_ENCODER,new PIDFCoefficients(P,0,0.01,FFar));
        launcherMotor2.setPIDFCoefficients(RUN_USING_ENCODER,new PIDFCoefficients(P,0,0.01,FFar));
        lt1.setPosition(.37);
        lt2.setPosition(.37); //.37 for far, .25 for close
        intakeMotor.setPower(1);
        sleep(500);
        boolean breaked = false;


        while (opModeIsActive()) {

            if(followerActive) {
                follower.update();
            }
            telemetry.addData("Elapsed time:",actionTimer.getElapsedTimeSeconds());
            telemetry.addLine(""+pathState);
            telemetry.update();
            /*
            if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 2) {
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
                            sleep(700);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            follower.followPath(goToPickup2);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;
             */
            switch (pathState) {
                case 0:
                    if(!follower.isBusy()){
                        follower.followPath(scorePreload);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 2.5) {
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
                            sleep(700);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            launcherMotor1.setVelocity(motorPowerClose / 60 * 28);
                            launcherMotor2.setVelocity(motorPowerClose / 60 * 28);
                            launcherMotor1.setPIDFCoefficients(RUN_USING_ENCODER,
                                    new PIDFCoefficients(P,0,0,FClose));
                            launcherMotor2.setPIDFCoefficients(RUN_USING_ENCODER,
                                    new PIDFCoefficients(P,0,0,FClose));
                            lt1.setPosition(.25);
                            lt2.setPosition(.25); //.37 for far, .25 for close
                            follower.followPath(goToPickup3);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;

                case 2:
                    if(!follower.isBusy()){
                        follower.followPath(grabPickup3);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;

                case 3:
                    if(!follower.isBusy()){
                        follower.followPath(clearClassifier);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 4:
                    if(!follower.isBusy()){
                        follower.followPath(scorePickup3);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 2.5) {
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
                            sleep(700);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            follower.followPath(goToPickup2);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;

                case 6:
                    if(!follower.isBusy()){
                        follower.followPath(grabPickup2);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 7:
                    if(!follower.isBusy()){
                        follower.followPath(scorePickup2);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 8:
                    if(!follower.isBusy()){
                        if(actionTimer.getElapsedTimeSeconds() < 2.5) {
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
                            sleep(700);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            launcherMotor1.setVelocity(motorPowerFar / 60 * 28);
                            launcherMotor2.setVelocity(motorPowerFar / 60 * 28);
                            launcherMotor1.setPIDFCoefficients(RUN_USING_ENCODER,new PIDFCoefficients(P,0,0.01,FFar));
                            launcherMotor2.setPIDFCoefficients(RUN_USING_ENCODER,new PIDFCoefficients(P,0,0.01,FFar));
                            lt1.setPosition(.37);
                            lt2.setPosition(.37); //.37 for far, .25 for close
                            follower.followPath(goToPickup1);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;
                case 9:
                    if(!follower.isBusy()){
                        follower.followPath(grabPickup1);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 10:
                    if(!follower.isBusy()){
                        follower.followPath(scorePickup1);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 11:
                    if(!follower.isBusy()){
                        if(actionTimer.getElapsedTimeSeconds() < 2.5) {
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
                            sleep(700);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            follower.followPath(getCorner);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;
                case 12:
                    if(!follower.isBusy()){
                        follower.followPath(getCorner2);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 13:
                    if(!follower.isBusy()){
                        follower.followPath(scoreCorner1);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 14:
                    if(!follower.isBusy()){
                        if(actionTimer.getElapsedTimeSeconds() < 2.5) {
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
                            sleep(700);
                            followerActive = true;
                            fires.primaryPos();
                            transferMotor.setPower(0);
                            follower.followPath(goToPickup1);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;

                case 15:
                    if(!follower.isBusy()){
                        breaked = true;
                    }
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
