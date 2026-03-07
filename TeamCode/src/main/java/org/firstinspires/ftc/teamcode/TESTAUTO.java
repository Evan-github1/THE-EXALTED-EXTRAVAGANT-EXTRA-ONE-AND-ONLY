package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.AutoConfig.BLUE_BALL1_START;
import static org.firstinspires.ftc.teamcode.AutoConfig.BLUE_CLOSE_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_BALL1_END;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_BALL1_START;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_BALL2_END;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_BALL2_START;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_BALL3_END;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_BALL3_START;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_CLOSE_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_FAR_CLEAR;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_FAR_CLEAR2;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_FAR_CORNER1;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_FAR_CORNER2;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_FAR_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_FAR_START;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
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
public class TESTAUTO extends Movable {
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
    private static Limelight3A limelight;
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
                .addPath(new BezierLine(RED_FAR_START,RED_FAR_SCORE))
                .setLinearHeadingInterpolation(RED_FAR_START.getHeading(),RED_FAR_SCORE.getHeading())
                .build();

        PathChain goToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_SCORE,RED_BALL3_START))
                .setLinearHeadingInterpolation(RED_FAR_SCORE.getHeading(),RED_BALL3_START.getHeading())
                .build();

        PathChain grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL3_START,RED_BALL3_END))
                .setLinearHeadingInterpolation(RED_BALL3_START.getHeading(),RED_BALL3_END.getHeading())
                .build();

        PathChain scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_CLEAR2,RED_CLOSE_SCORE))
                .setLinearHeadingInterpolation(RED_FAR_CLEAR2.getHeading(),RED_CLOSE_SCORE.getHeading())
                .build();

        PathChain clearClassifier = follower.pathBuilder()
                .addPath(new BezierCurve(RED_BALL3_END, RED_FAR_CLEAR, RED_FAR_CLEAR2))
                .setConstantHeadingInterpolation(0)
                .build();

        PathChain goToPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_SCORE,RED_BALL2_START))
                .setLinearHeadingInterpolation(RED_FAR_SCORE.getHeading(),RED_BALL2_START.getHeading())
                .build();

        PathChain grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL2_START,RED_BALL2_END))
                .setLinearHeadingInterpolation(RED_BALL2_START.getHeading(),RED_BALL2_END.getHeading())
                .build();

        PathChain scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL2_END,RED_CLOSE_SCORE))
                .setLinearHeadingInterpolation(RED_BALL2_END.getHeading(),RED_CLOSE_SCORE.getHeading())
                .build();

        PathChain goToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_SCORE,RED_BALL1_START))
                .setLinearHeadingInterpolation(RED_FAR_SCORE.getHeading(),RED_BALL1_START.getHeading())
                .build();

        PathChain grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL1_START,RED_BALL1_END))
                .setLinearHeadingInterpolation(RED_BALL1_START.getHeading(),RED_BALL1_END.getHeading())
                .build();

        PathChain scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_BALL1_END,RED_FAR_SCORE))
                .setLinearHeadingInterpolation(RED_BALL1_END.getHeading(),RED_FAR_SCORE.getHeading())
                .build();

        PathChain getCorner = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_SCORE, RED_FAR_CORNER1))
                .setLinearHeadingInterpolation(RED_FAR_SCORE.getHeading(), RED_FAR_CORNER1.getHeading())
                .build();

        PathChain scoreCorner1 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_CORNER2,RED_FAR_SCORE))
                .setLinearHeadingInterpolation(RED_FAR_CORNER2.getHeading(), RED_FAR_SCORE.getHeading())
                .build();

        PathChain getCorner2 = follower.pathBuilder()
                .addPath(new BezierLine(RED_FAR_CORNER1,RED_FAR_CORNER2))
                .setConstantHeadingInterpolation(RED_FAR_CORNER1.getHeading())
                .build();
        PathChain goTEST = follower.pathBuilder()
                        .addPath(new BezierCurve(BLUE_BALL1_START,BLUE_CLOSE_SCORE,RED_BALL1_START))
                                .setLinearHeadingInterpolation(BLUE_BALL1_START.getHeading(),RED_BALL2_START.getHeading())
                                        .build();

        follower.setStartingPose(BLUE_BALL1_START);

        pathState = 0;
        AutoConfig.isRed = true;

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
        isAimed = false;


        while (opModeIsActive()) {
            follower.update();
            switch(pathState) {
                case 0:
                    if (!follower.isBusy()) {
                        follower.followPath(goTEST);
                        pathState++;
                    }
                case 1:
                    telemetry.addData("X:", follower.getPose().getX());
                    telemetry.addData("Y:",follower.getPose().getY());
                    telemetry.update();

            }
            if(isAimed){
                break;
            }
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
