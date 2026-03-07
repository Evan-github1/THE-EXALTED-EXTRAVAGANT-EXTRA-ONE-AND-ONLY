package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotWidth;
import static org.firstinspires.ftc.teamcode.AutoConfig.*;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
public class OneTickAndHPZone_BLUE extends Movable {
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
                .addPath(new BezierLine(BLUE_FAR_START,BLUE_FAR_SCORE))
                .setLinearHeadingInterpolation(BLUE_FAR_START.getHeading(),BLUE_FAR_SCORE.getHeading())
                .build();

        PathChain clearClassifier = follower.pathBuilder()
                .addPath(new BezierCurve(BLUE_BALL3_END, BLUE_FAR_CLEAR2, BLUE_FAR_CLEAR))
                .setConstantHeadingInterpolation(Math.PI)
                .build();

        PathChain goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BLUE_FAR_SCORE,BLUE_BALL1_START))
                .setLinearHeadingInterpolation(BLUE_FAR_SCORE.getHeading(),BLUE_BALL1_START.getHeading())
                .build();

        PathChain grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BLUE_BALL1_START,BLUE_BALL1_END))
                .setLinearHeadingInterpolation(BLUE_BALL1_START.getHeading(),BLUE_BALL1_END.getHeading())
                .build();

        PathChain scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(BLUE_BALL1_END,BLUE_FAR_SCORE))
                .setLinearHeadingInterpolation(BLUE_BALL1_END.getHeading(),BLUE_FAR_SCORE.getHeading())
                .build();

        PathChain getCorner = follower.pathBuilder()
                .addPath(new BezierLine(BLUE_FAR_SCORE, BLUE_FAR_CORNER1))
                .setLinearHeadingInterpolation(BLUE_FAR_SCORE.getHeading(), BLUE_FAR_CORNER1.getHeading())
                .build();

        PathChain scoreCorner1 = follower.pathBuilder()
                .addPath(new BezierLine(BLUE_FAR_CORNER2,BLUE_FAR_SCORE))
                .setLinearHeadingInterpolation(BLUE_FAR_CORNER2.getHeading(), BLUE_FAR_SCORE.getHeading())
                .build();

        PathChain getCorner2 = follower.pathBuilder()
                .addPath(new BezierLine(BLUE_FAR_CORNER1,BLUE_FAR_CORNER2))
                .setConstantHeadingInterpolation(BLUE_FAR_CORNER1.getHeading())
                .build();

        PathChain cornerDirect = follower.pathBuilder()
                .addPath(new BezierLine(BLUE_FAR_SCORE,BLUE_FAR_CORNER_DIRECT))
                .setConstantHeadingInterpolation(BLUE_FAR_CORNER_DIRECT.getHeading())
                .build();

        PathChain backFromCornerDirect = follower.pathBuilder()
                .addPath(new BezierLine(BLUE_FAR_CORNER_DIRECT,BLUE_FAR_SCORE))
                .setLinearHeadingInterpolation(BLUE_FAR_CORNER_DIRECT.getHeading(),BLUE_FAR_SCORE.getHeading())
                .build();

        follower.setStartingPose(BLUE_FAR_START);

        pathState = 0;
        AutoConfig.isRed = false;

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
            AutoConfig.lastAutoEndPose = follower.getPose();
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
                        pathState=8;
                    }
                    actionTimer.resetTimer();
                    break;
                case 8:
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
                            follower.followPath(cornerDirect,.7,true);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;

                case 15:
                    if(!follower.isBusy()){
                        follower.followPath(backFromCornerDirect);
                        pathState++;
                    }
                    actionTimer.resetTimer();

                case 16:
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
                            follower.followPath(cornerDirect,.7,true);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;

                case 17:
                    if(!follower.isBusy()){
                        follower.followPath(backFromCornerDirect);
                        pathState++;
                    }
                    actionTimer.resetTimer();

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
                            follower.followPath(goToPickup1);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;

                case 19:
                    if(!follower.isBusy()){
                        AutoConfig.isRed = false;
                        AutoConfig.lastAutoEndPose = follower.getPose();
                        breaked = true;
                    }
                    break;
            }
            if(breaked) break;
        }

    }

    private boolean autoAim(){
        double goalX = 6;
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
