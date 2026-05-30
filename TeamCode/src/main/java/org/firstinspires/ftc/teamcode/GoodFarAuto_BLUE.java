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
public class GoodFarAuto_BLUE extends Movable {
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private static DcMotorEx intakeMotor, launcherMotor1, launcherMotor2,transferMotor;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static DoubleSwitchedServo fires;
    private Follower follower;
    private int iterations;
    private static Limelight3A limelight;
    private static double motorPowerFar, motorPowerClose;
    private static double rpm, tps, targetRPM, P, FClose, FFar, currentTargetRPM;
    private static PIDFCoefficients pidfCoefficients;
    private static double pastError;
    private boolean followerActive;
    private static PIDFController aimPID;

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
        motorPowerClose = 2200;
        motorPowerFar = 3050; //from 4800
        followerActive = true;
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.pipelineSwitch(0);
        targetRPM = motorPowerFar;
        aimPID = new PIDFController(new com.pedropathing.control.PIDFCoefficients(1.2,0,0.05,0.025));
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
                .addPath(new BezierCurve(BLUE_FAR_SCORE, BLUE_FAR_CORNER1,BLUE_FAR_CORNER2))
                .setTangentHeadingInterpolation()
                .build();

        PathChain scoreCorner = follower.pathBuilder()
                .addPath(new BezierLine(BLUE_FAR_CORNER2,BLUE_FAR_SCORE))
                .setLinearHeadingInterpolation(BLUE_FAR_CORNER2.getHeading(), BLUE_FAR_SCORE.getHeading())
                .build();

        PathChain getLandingZone = follower.pathBuilder()
                .addPath(new BezierCurve(BLUE_FAR_SCORE,GET_LANDING_ZONE_BLUE1,GET_LANDING_ZONE_BLUE2))
                .setTangentHeadingInterpolation()
                .build();

        PathChain scoreLandingZone = follower.pathBuilder()
                .addPath(new BezierLine(GET_LANDING_ZONE_BLUE2,BLUE_FAR_SCORE))
                .setLinearHeadingInterpolation(GET_LANDING_ZONE_BLUE2.getHeading(), BLUE_FAR_SCORE.getHeading())
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
        sleep(2000);
        boolean breaked = false;


        while (opModeIsActive()) {

            follower.update();
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
                        actionTimer.resetTimer();
                    }
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 4) {
                            try {
                                if (iterations == 0) {
                                    pastError = 0;
                                }
                                pastError = LeBotsEyes(pastError, false);
                                LeBotsEyes(pastError, true);
                                iterations++;
                            } catch(Exception ignored){}
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
                            follower.followPath(goToPickup1);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;

                case 2:
                    if(!follower.isBusy()){
                        follower.followPath(grabPickup1);
                        pathState++;
                        actionTimer.resetTimer();
                    }
                    break;

                case 3:
                    if(!follower.isBusy()){
                        follower.followPath(scorePickup1);
                        pathState++;
                        actionTimer.resetTimer();
                    }
                    break;

                case 4:
                    if (!follower.isBusy()) {
                        if(actionTimer.getElapsedTimeSeconds() < 2) {
                            try {
                                if (iterations == 0) {
                                    pastError = 0;
                                }
                                pastError = LeBotsEyes(pastError, false);
                                LeBotsEyes(pastError, true);
                                iterations++;
                            } catch(Exception ignored){}
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
                            follower.followPath(getCorner);
                            pathState++;
                            actionTimer.resetTimer();
                        }
                    }
                    break;

                case 5:
                    if (!follower.isBusy() || follower.isRobotStuck()) {
                        follower.followPath(scoreCorner);
                        pathState = 4;
                        actionTimer.resetTimer();
                    }
                    break;
            }
        }

    }

    private double LeBotsEyes(double pastError, boolean adjustMotor){
        double desiredX;
        telemetry.addData("D", true);
        LLResultTypes.FiducialResult yes = detectTagSelective(limelight,telemetry);
        if(yes != null){
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

    private boolean autoAim() {
        double turn;
        double error = 100000;
        double goalX;
        double goalY = 140;
        if(AutoConfig.isRed){
            goalX=140;
        } else{
            goalX=7;//for better aiming?
        }
        //calculate angle from robot to target, add pi to get angle robot needs to face to aim at target (since launcher is on back of robot)
        double targetAngle = Math.atan2(goalY - follower.getPose().getY(), goalX - follower.getPose().getX()) + Math.PI;

        //calculate error between target angle and current angle
        error = targetAngle - follower.getPose().getHeading();

        //convert error to range [-pi, pi] so that robot turns the shortest distance to target
        while(error>Math.PI){error-=2*Math.PI;}
        while(error<-Math.PI){error+=2*Math.PI;}

        //run PID
        aimPID.updateError(error);
        turn = aimPID.run();
        if(Math.abs(error) < Math.toRadians(1)){
            turn = 0;
        }
        //drive wheels
        follower.setTeleOpDrive(0,0,turn);
        //isAimed check basically
        //checks for is within 0.05 degrees of target angle and not rotating too fast
        return Math.abs(error) < Math.toRadians(1.5) && Math.abs(follower.getAngularVelocity()) < Math.toRadians(1.5);
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
