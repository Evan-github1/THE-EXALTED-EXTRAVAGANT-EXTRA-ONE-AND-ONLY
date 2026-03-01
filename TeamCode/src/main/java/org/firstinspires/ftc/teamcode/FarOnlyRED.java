package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotWidth;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
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
public class FarOnlyRED extends Movable {
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


    private final Pose startPose = new Pose(96, 0 + robotLength()/2, Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(88.6,11.7,Math.toRadians(-115.5));
    private final Pose ball1PickupStart = new Pose(98,31,0);
    private final Pose ball1PickupEnd = new Pose(133,31,0);
    private final Pose ball2PickupStart = new Pose(98,55,0);
    private final Pose ball2PickupEnd = new Pose(130,55,0);
    private final Pose ball3PickupStart = new Pose(98,79,0);
    private final Pose ball3PickupEnd = new Pose(128,79,0);
    private final Pose clearClassifier1 = new Pose(128,67,0);
    private final Pose clearClassifier2 = new Pose(118,67,0);
    private final Pose pickUpCorner1 = new Pose(141.5-robotWidth()/2,30+robotLength()/2,Math.toRadians(-90));
    private final Pose pickUpCorner2 = new Pose(141.5-robotWidth()/2,2+robotLength()/2+3,Math.toRadians(-90));
    private final Pose closeShoot = new Pose(85,85,Math.PI+Math.PI/4);
    private final Pose corner3 = new Pose(141.5-robotLength()/2 + 2,robotWidth()/2+2,0);

    public void runOpMode() throws InterruptedException {
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
        motorPowerFar = 3400; //from 4800
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
        follower.setStartingPose(startPose);


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

        PathChain scoreToBall1Pickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,ball1PickupStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), ball1PickupStart.getHeading())
                .build();

        PathChain grabBall1 = follower.pathBuilder()
                .addPath(new BezierLine(ball1PickupStart,ball1PickupEnd))
                .setConstantHeadingInterpolation(ball1PickupStart.getHeading())
                .build();

        PathChain scoreBall1 = follower.pathBuilder()
                .addPath(new BezierLine(ball1PickupEnd,scorePose))
                .setLinearHeadingInterpolation(ball1PickupEnd.getHeading(), scorePose.getHeading())
                .build();

        PathChain scoreToBall2Pickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,ball2PickupStart))
                .setLinearHeadingInterpolation(scorePose.getHeading(), ball2PickupStart.getHeading())
                .build();

        PathChain grabBall2Pickup = follower.pathBuilder()
                .addPath(new BezierLine(ball2PickupStart,ball2PickupEnd))
                .setConstantHeadingInterpolation(ball2PickupEnd.getHeading())
                .build();

        PathChain scoreBall2 = follower.pathBuilder()
                .addPath(new BezierLine(ball2PickupEnd,scorePose))
                .setLinearHeadingInterpolation(ball2PickupEnd.getHeading(), scorePose.getHeading())
                .build();

        PathChain scoreToHPPickup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,pickUpCorner1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUpCorner1.getHeading())
                .build();

        PathChain grabHPPickup = follower.pathBuilder()
                .addPath(new BezierLine(pickUpCorner1,pickUpCorner2))
                .setConstantHeadingInterpolation(pickUpCorner1.getHeading())
                .build();

        PathChain scoreHPPickup = follower.pathBuilder()
                .addPath(new BezierLine(pickUpCorner2,scorePose))
                .setLinearHeadingInterpolation(pickUpCorner2.getHeading(), scorePose.getHeading())
                .build();

        PathChain cornerGrab = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,corner3))
                .setConstantHeadingInterpolation(corner3.getHeading())
                .build();

        PathChain score = follower.pathBuilder()
                .addPath(new BezierLine(corner3,scorePose))
                .setLinearHeadingInterpolation(corner3.getHeading(),scorePose.getHeading())
                .build();

        waitForStart();
        launcherMotor1.setVelocity((motorPowerFar+100) / 60 * 28);
        launcherMotor2.setVelocity((motorPowerFar+100) / 60 * 28);
        launcherMotor1.setPIDFCoefficients(RUN_USING_ENCODER,new PIDFCoefficients(P,0,0.01,FFar));
        launcherMotor2.setPIDFCoefficients(RUN_USING_ENCODER,new PIDFCoefficients(P,0,0.01,FFar));
        lt1.setPosition(.37);
        lt2.setPosition(.37); //.37 for far, .25 for close
        intakeMotor.setPower(1);
        sleep(500);

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
                    if (!follower.isBusy()) {
                        follower.followPath(scorePreload);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        if (actionTimer.getElapsedTimeSeconds() < 2) {
                            try {
                                if (iterations == 0) {
                                    pastError = 0;
                                    LeBotsEyes(pastError, true);
                                } else {
                                    pastError = LeBotsEyes(pastError, false);
                                    LeBotsEyes(pastError, true);
                                }
                                iterations++;
                            } catch (Exception ignored) {
                            }
                            ;
                        } else {
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
                            follower.followPath(cornerGrab);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;

                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(score);
                        actionTimer.resetTimer();
                        pathState=1;
                    }else if(actionTimer.getElapsedTimeSeconds() > 4.5){
                        follower.followPath(new Path(new BezierLine(follower.getPose(), scorePose)));
                        actionTimer.resetTimer();
                        pathState = 1;
                    }
                    break;
            }
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
            smoothedError = smoothedError/10;
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
