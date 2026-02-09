package org.firstinspires.ftc.teamcode.Bluebots;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;

@Autonomous
public class TheDeathOfBlueFarCrappierPedroPathingGPTed
        extends TheDeathOfPedroPathing
        implements LimelightTags {

    /* ================= POSES ================= */

    private static final Pose startPose =
            new Pose(48 + ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2,
                    Math.toRadians(180));

    private static final Pose startCollectFirst =
            new Pose(48 + ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 26,
                    Math.toRadians(180));

    private static final Pose endCollectFirst =
            new Pose(ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 26,
                    Math.toRadians(180));

    private static final Pose startCollectSecond =
            new Pose(48 + ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 48,
                    Math.toRadians(180));

    private static final Pose endCollectSecond =
            new Pose(ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 48,
                    Math.toRadians(180));

    private static final Pose shootClosePos =
            new Pose(48,
                    ROBOT_WIDTH / 2 + 96,
                    Math.toRadians(-45));

    /* ================= PATHS ================= */

    private PathChain startToSecond;
    private PathChain secondCollect;
    private PathChain goToCloseShoot;
    private PathChain closeToFirst;
    private PathChain firstCollectLate;

    /* ================= SHOOTER ================= */

    private enum ShotPhase { LIFT, DROP, FEED }

    private ShotPhase shotPhase = ShotPhase.LIFT;
    private int shotCount = 0;

    /* ================= AUTO STATES ================= */

    private enum AutoState {

        PRELOAD_SPINUP,
        PRELOAD_WAIT,
        PRELOAD_SHOOT,

        PATH_TO_SECOND,
        WAIT_SECOND,
        COLLECT_SECOND,

        PATH_TO_CLOSE,
        WAIT_CLOSE,

        CLOSE_SPINUP,
        CLOSE_WAIT,
        CLOSE_SHOOT,

        PATH_TO_FIRST_LATE,
        WAIT_FIRST_LATE,
        COLLECT_FIRST_LATE,

        DONE
    }

    private AutoState autoState = AutoState.PRELOAD_SPINUP;
    private final ElapsedTime timer = new ElapsedTime();

    /* ================= RUN ================= */

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            follower.update();

            switch (autoState) {

                /* ================= PRELOAD ================= */

                case PRELOAD_SPINUP:
                    outtakeMotor.setVelocity(1775);
                    hoodServo.setPosition(.3);
                    moveMotorToPosition(-305, .25);
                    timer.reset();
                    autoState = AutoState.PRELOAD_WAIT;
                    break;

                case PRELOAD_WAIT:
                    if (rpmReady(1775) && turretReady()) {
                        shotCount = 0;
                        shotPhase = ShotPhase.LIFT;
                        intakeMotor.setPower(0);
                        timer.reset();
                        autoState = AutoState.PRELOAD_SHOOT;
                    }
                    break;

                case PRELOAD_SHOOT:
                    runTripleShot(AutoState.PATH_TO_SECOND);
                    break;

                /* ================= SECOND ROW ================= */

                case PATH_TO_SECOND:
                    follower.followPath(startToSecond);
                    autoState = AutoState.WAIT_SECOND;
                    break;

                case WAIT_SECOND:
                    if (!follower.isBusy())
                        autoState = AutoState.COLLECT_SECOND;
                    break;

                case COLLECT_SECOND:
                    follower.followPath(secondCollect, .5, true);
                    autoState = AutoState.PATH_TO_CLOSE;
                    break;

                /* ================= CLOSE ================= */

                case PATH_TO_CLOSE:
                    if (!follower.isBusy()) {
                        follower.followPath(goToCloseShoot);
                        autoState = AutoState.WAIT_CLOSE;
                    }
                    break;

                case WAIT_CLOSE:
                    if (!follower.isBusy())
                        autoState = AutoState.CLOSE_SPINUP;
                    break;

                case CLOSE_SPINUP:
                    outtakeMotor.setVelocity(1500);
                    hoodServo.setPosition(.5);
                    moveMotorToPosition(0, .3);
                    timer.reset();
                    autoState = AutoState.CLOSE_WAIT;
                    break;

                case CLOSE_WAIT:
                    if (rpmReady(1500)) {
                        shotCount = 0;
                        shotPhase = ShotPhase.LIFT;
                        intakeMotor.setPower(0);
                        timer.reset();
                        autoState = AutoState.CLOSE_SHOOT;
                    }
                    break;

                case CLOSE_SHOOT:
                    runTripleShotRightOnly(AutoState.PATH_TO_FIRST_LATE);
                    break;

                /* ================= FIRST ROW LATE ================= */

                case PATH_TO_FIRST_LATE:
                    follower.followPath(closeToFirst);
                    autoState = AutoState.WAIT_FIRST_LATE;
                    break;

                case WAIT_FIRST_LATE:
                    if (!follower.isBusy())
                        autoState = AutoState.COLLECT_FIRST_LATE;
                    break;

                case COLLECT_FIRST_LATE:
                    follower.followPath(firstCollectLate, .5, true);
                    autoState = AutoState.DONE;
                    break;

                /* ================= DONE ================= */

                case DONE:
                    follower.breakFollowing();
                    break;
            }

            telemetry.addData("State", autoState);
            telemetry.addData("Shots", shotCount);
            telemetry.addData("RPM", outtakeMotor.getVelocity());
            telemetry.update();
        }
    }

    /* ================= SHOOTING ================= */

    private void runTripleShot(AutoState nextState) {

        final double LIFT_TIME = 0.9;
        final double FEED_TIME = 0.45;

        if (shotCount >= 3) {
            wipersL.primaryPos();
            wipersR.primaryPos();
            intakeMotor.setPower(1);
            autoState = nextState;
            return;
        }

        switch (shotPhase) {

            case LIFT:
                intakeMotor.setPower(0);

                if (shotCount == 0 || shotCount == 2)
                    wipersR.secondaryPos();
                else
                    wipersL.secondaryPos();

                timer.reset();
                shotPhase = ShotPhase.DROP;
                break;

            case DROP:
                if (timer.seconds() > LIFT_TIME) {

                    wipersL.primaryPos();
                    wipersR.primaryPos();
                    intakeMotor.setPower(1);

                    timer.reset();
                    shotPhase = ShotPhase.FEED;
                }
                break;

            case FEED:
                if (timer.seconds() > FEED_TIME) {

                    intakeMotor.setPower(0);
                    shotCount++;

                    shotPhase = ShotPhase.LIFT;
                    timer.reset();
                }
                break;
        }
    }

    private void runTripleShotRightOnly(AutoState nextState) {

        final double LIFT_TIME = 0.9;
        final double FEED_TIME = 0.45;

        if (shotCount >= 3) {

            wipersL.primaryPos();
            wipersR.primaryPos();
            intakeMotor.setPower(.6);

            autoState = nextState;
            return;
        }

        switch (shotPhase) {

            case LIFT:
                intakeMotor.setPower(0);
                wipersR.secondaryPos();
                timer.reset();
                shotPhase = ShotPhase.DROP;
                break;

            case DROP:
                if (timer.seconds() > LIFT_TIME) {

                    wipersR.primaryPos();
                    intakeMotor.setPower(.6);

                    timer.reset();
                    shotPhase = ShotPhase.FEED;
                }
                break;

            case FEED:
                if (timer.seconds() > FEED_TIME) {

                    intakeMotor.setPower(0);
                    shotCount++;

                    shotPhase = ShotPhase.LIFT;
                    timer.reset();
                }
                break;
        }
    }

    /* ================= HELPERS ================= */

    private boolean rpmReady(double target) {
        return Math.abs(outtakeMotor.getVelocity() - target) < 75
                || timer.seconds() > 2.5;
    }

    private boolean turretReady() {
        return !swivelTurretMotor.isBusy()
                || timer.seconds() > 3.5;
    }

    private void moveMotorToPosition(int target, double power) {

        swivelTurretMotor.setTargetPosition(target);
        swivelTurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swivelTurretMotor.setPower(power);
    }

    /* ================= PATHS ================= */

    private void buildPaths() {

        startToSecond = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startCollectSecond))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        startCollectSecond.getHeading())
                .build();

        secondCollect = follower.pathBuilder()
                .addPath(new BezierLine(startCollectSecond, endCollectSecond))
                .setLinearHeadingInterpolation(
                        startCollectSecond.getHeading(),
                        endCollectSecond.getHeading())
                .build();

        goToCloseShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        endCollectSecond,
                        new Pose(24, 60),
                        shootClosePos))
                .setLinearHeadingInterpolation(
                        endCollectSecond.getHeading(),
                        shootClosePos.getHeading())
                .build();

        closeToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(
                        shootClosePos,
                        new Pose(36, 48),
                        startCollectFirst))
                .setLinearHeadingInterpolation(
                        shootClosePos.getHeading(),
                        startCollectFirst.getHeading())
                .build();

        firstCollectLate = follower.pathBuilder()
                .addPath(new BezierLine(startCollectFirst, endCollectFirst))
                .setLinearHeadingInterpolation(
                        startCollectFirst.getHeading(),
                        endCollectFirst.getHeading())
                .build();
    }

    /* ================= LIMELIGHT ================= */

    @Override public void tag20() {}
    @Override public void tag21() {}
    @Override public void tag22() {}
    @Override public void tag23() {}
    @Override public void tag24() {}
    @Override public void nothing() {}
}
