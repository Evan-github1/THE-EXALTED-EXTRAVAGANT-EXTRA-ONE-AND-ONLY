package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.AutoConfig.*;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@Autonomous
public class CloseTwelveBallAuto_RED extends Movable {
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
    private int iterations;
    private static PIDFController aimPID;
    private boolean isAimed;
    private static double motorPowerFar, motorPowerClose;
    private static double rpm, tps, targetRPM, P, FClose, FFar, currentTargetRPM;
    private static PIDFCoefficients pidfCoefficients;
    private boolean followerActive;


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
        motorPowerClose = AutoConfig.motorPowerClose;
        motorPowerFar = AutoConfig.motorPowerFar; //from 4800
        followerActive = true;
        aimPID = new PIDFController(new com.pedropathing.control.PIDFCoefficients(1.2,0,0.05,0.025));
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
                .addPath(new BezierLine(RED_CLOSE_START,RED_CLOSE_SCORE))
                .setLinearHeadingInterpolation(RED_CLOSE_START.getHeading(),RED_CLOSE_SCORE.getHeading())
                .build();

        PathChain clearClassifier = follower.pathBuilder()
                .addPath(new BezierCurve(RED_BALL2_END, RED_FAR_CLEAR, RED_FAR_CLEAR2))
                .setLinearHeadingInterpolation(RED_BALL2_END.getHeading(),RED_FAR_CLEAR2.getHeading(),0.7)
                .build();

        PathChain cornerDirect = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_SCORE,RED_FAR_CORNER_DIRECT))
                .setConstantHeadingInterpolation(RED_FAR_CORNER_DIRECT.getHeading())
                .build();

        PathChain backFromCornerDirect = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_CORNER_DIRECT,RED_FAR_SCORE))
                .setLinearHeadingInterpolation(RED_FAR_CORNER_DIRECT.getHeading(),RED_FAR_SCORE.getHeading())
                .build();

        PathChain goToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RED_CLOSE_SCORE,RED_BALL3_START))
                .setLinearHeadingInterpolation(RED_CLOSE_SCORE.getHeading(), RED_BALL3_START.getHeading())
                .build();

        PathChain grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL3_START,RED_BALL3_END))
                .setLinearHeadingInterpolation(RED_BALL3_START.getHeading(), RED_BALL3_END.getHeading())
                .build();

        PathChain scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL3_END,RED_CLOSE_SCORE))
                .setLinearHeadingInterpolation(RED_BALL3_END.getHeading(), RED_CLOSE_SCORE.getHeading())
                .build();

        PathChain goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RED_CLOSE_SCORE,RED_BALL2_START))
                .setLinearHeadingInterpolation(RED_CLOSE_SCORE.getHeading(), RED_BALL2_START.getHeading())
                .build();

        PathChain grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL2_START,RED_BALL2_END))
                .setLinearHeadingInterpolation(RED_BALL2_START.getHeading(), RED_BALL2_END.getHeading())
                .build();

        PathChain scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_CLEAR2,RED_CLOSE_SCORE))
                .setLinearHeadingInterpolation(RED_FAR_CLEAR2.getHeading(), RED_CLOSE_SCORE.getHeading())
                .build();

        PathChain goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_CLOSE_SCORE,RED_BALL1_START))
                .setLinearHeadingInterpolation(RED_CLOSE_SCORE.getHeading() , RED_BALL1_START.getHeading())
                .build();

        PathChain grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL1_START,RED_BALL1_END))
                .setLinearHeadingInterpolation(RED_BALL1_START.getHeading(), RED_BALL1_END.getHeading())
                .build();

        PathChain scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL1_END,RED_CLOSE_SCORE))
                .setLinearHeadingInterpolation(RED_BALL1_END.getHeading(), RED_CLOSE_SCORE.getHeading())
                .build();

        follower.setStartingPose(RED_CLOSE_START);

        pathState = 0;
        AutoConfig.isRed = true;
        AutoConfig.lastAutoEndPose = RED_CLOSE_START;

        waitForStart();
        launcherMotor1.setVelocity(motorPowerClose / 60 * 28);
        launcherMotor2.setVelocity(motorPowerClose / 60 * 28);
        launcherMotor1.setPIDFCoefficients(RUN_USING_ENCODER,new PIDFCoefficients(P,0,0.01,FFar));
        launcherMotor2.setPIDFCoefficients(RUN_USING_ENCODER,new PIDFCoefficients(P,0,0.01,FFar));
        lt1.setPosition(.25);
        lt2.setPosition(.25); //.37 for far, .25 for close
        intakeMotor.setPower(1);
        sleep(500);
        boolean breaked = false;


        while (opModeIsActive()) {
            telemetry.addData("X:", follower.getPose().getX());
            telemetry.addData("Y:",follower.getPose().getY());
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
                                }
                                pastError = LeBotsEyes(pastError, false);
                                LeBotsEyes(pastError, true);
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
                        }
                    }
                    break;
             */
            switch (pathState) {
                case 0:
                    if(!follower.isBusy()){
                        follower.followPath(scorePreload);
                        pathState=7;
                    }
                    actionTimer.resetTimer();
                    break;
                case 7:
                    if(!follower.isBusy()){
                        if(iterations == 0){
                            follower.startTeleopDrive(true);
                            isAimed = false;
                            iterations++;
                        }
                        if(!isAimed && actionTimer.getElapsedTimeSeconds() < 3) {
                            isAimed = autoAim();
                        }else {
                            follower.setTeleOpDrive(0,0,0);
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
                    if(!follower.isBusy()){
                        follower.followPath(grabPickup3);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 12:
                    if(!follower.isBusy()){
                        follower.followPath(clearClassifier);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 9:
                    if(!follower.isBusy()){
                        follower.followPath(scorePickup3);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 10:
                    if(!follower.isBusy()){
                        if(iterations == 0){
                            follower.startTeleopDrive(true);
                            isAimed = false;
                            iterations++;
                        }
                        if(!isAimed && actionTimer.getElapsedTimeSeconds() < 3) {
                            isAimed = autoAim();
                        }else {
                            follower.setTeleOpDrive(0,0,0);
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
                    if(!follower.isBusy()){
                        follower.followPath(grabPickup2);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 13:
                    if(!follower.isBusy()){
                        follower.followPath(scorePickup2);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 14:
                    if(!follower.isBusy()){
                        if(iterations == 0){
                            follower.startTeleopDrive(true);
                            isAimed = false;
                            iterations++;
                        }
                        if(!isAimed && actionTimer.getElapsedTimeSeconds() < 3) {
                            isAimed = autoAim();
                        }else {
                            follower.setTeleOpDrive(0,0,0);
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
                case 15:
                    if(!follower.isBusy()){
                        follower.followPath(grabPickup1);
                        pathState++;
                    }
                    actionTimer.resetTimer();
                    break;
                case 16:
                    if(!follower.isBusy()){
                        follower.followPath(scorePickup1);
                        pathState+=2;
                    }
                    actionTimer.resetTimer();
                    break;
                case 18:
                    if(!follower.isBusy()){
                        if(iterations == 0){
                            follower.startTeleopDrive(true);
                            isAimed = false;
                            iterations++;
                        }
                        if(!isAimed && actionTimer.getElapsedTimeSeconds() < 3) {
                            isAimed = autoAim();
                        }else {
                            follower.setTeleOpDrive(0,0,0);
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

                case 19:
                    if(!follower.isBusy()){
                        AutoConfig.isRed = true;
                        AutoConfig.lastAutoEndPose = follower.getPose();
                        breaked = true;
                    }
                    break;
            }
            if(breaked) break;
        }
        AutoConfig.lastAutoEndPose = follower.getPose();

    }

    private boolean autoAim(){
        double goalX = 141.5;
        double goalY = 140;
        double targetAngle = Math.atan2(goalY - follower.getPose().getY(), goalX - follower.getPose().getX()) + Math.PI;
        double error = targetAngle - follower.getPose().getHeading();
        while(error > Math.PI){ error -= 2*Math.PI; }
        while(error < -Math.PI){ error += 2*Math.PI; }
        aimPID.updateError(error);
        double turn = aimPID.run();
        follower.setTeleOpDrive(0, 0, turn);
        return Math.abs(error) < Math.toRadians(0.5) && Math.abs(follower.getAngularVelocity()) < Math.toRadians(0.5);
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
