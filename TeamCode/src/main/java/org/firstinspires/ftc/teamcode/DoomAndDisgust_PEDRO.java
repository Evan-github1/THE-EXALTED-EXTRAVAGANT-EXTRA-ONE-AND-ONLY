package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotWidth;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;

@Autonomous
public class DoomAndDisgust_PEDRO extends Movable {

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private static DcMotor intakeMotor, launcherMotor1, launcherMotor2;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static TripleSwitchedServo fires;
    private static Servo fork;
    private static DoubleSwitchedServo forks;
    private Follower follower;

    private final Pose startPose = new Pose(96, 0 + robotLength()/2, Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(84,12,Math.toRadians(-126));
    private final Pose ball1PickupStart = new Pose(100,36,0);
    private final Pose ball1PickupEnd = new Pose(120,36,0);
    private final Pose ball2PickupStart = new Pose(100,60,0);
    private final Pose ball2PickupEnd = new Pose(120,60,0);
    private final Pose ball3PickupStart = new Pose(100,84,0);
    private final Pose ball3PickupEnd = new Pose(120,84,0);

    private PathChain scorePreload, goToPickup1, grabPickup1, scorePickup1, goToPickup2, grabPickup2, scorePickup2, goToPickup3, grabPickup3, scorePickup3;
    public void runOpMode() throws InterruptedException{


        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotor.class,"INT");
        lt1 = hardwareMap.get(Servo.class,"LT1");
        lt2 = hardwareMap.get(Servo.class,"LT2");
        fire = hardwareMap.get(Servo.class, "FIRE");
        fires = new TripleSwitchedServo(fire,.55,.49,.35);
        launcherMotor1 = hardwareMap.get(DcMotor.class,"LAU1");
        launcherMotor2 = hardwareMap.get(DcMotor.class,"LAU2");
        fork = hardwareMap.get(Servo.class,"FORK");
        forks = new DoubleSwitchedServo(fork,0.23,0.75);
        follower = createFollower(hardwareMap);

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                .addPath(new BezierLine(ball3PickupEnd, scorePose))
                .setLinearHeadingInterpolation(ball3PickupEnd.getHeading(), scorePose.getHeading())
                .build();

        follower.setStartingPose(startPose);

        int pathState = 0;

        waitForStart();



        while (opModeIsActive()) {

            follower.update();

            switch (pathState) {

                case 0:
                    follower.followPath(scorePreload);
                    pathState++;
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(goToPickup1);
                        pathState++;
                    }
                    break;

                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup1);
                        pathState++;
                    }
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup1);
                        pathState++;
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(goToPickup2);
                        pathState++;
                    }
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup2);
                        pathState++;
                    }
                    break;
                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup2);
                        pathState++;
                    }
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        follower.followPath(goToPickup3);
                        pathState++;
                    }
                    break;
                case 8:
                    if (!follower.isBusy()) {
                        follower.followPath(grabPickup3);
                        pathState++;
                    }
                    break;
                case 9:
                    if (!follower.isBusy()) {
                        follower.followPath(scorePickup3);
                        pathState++;
                    }
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
