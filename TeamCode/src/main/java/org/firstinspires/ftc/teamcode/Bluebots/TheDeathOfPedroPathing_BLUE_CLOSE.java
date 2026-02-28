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
public class TheDeathOfPedroPathing_BLUE_CLOSE
        extends TheDeathOfPedroPathing
        implements LimelightTags {

    /* ================= POSES ================= */

    private static final Pose startPose =
            new Pose(24 - 3.5,
                    (5 * 24) + 3.25,
                    Math.toRadians(-40));

    // the pose to shoot close. the bot should go to this position 3 times during auto.
    private static final Pose shootClosePos =
            new Pose(60,
                    ROBOT_WIDTH / 2 + 84,
                    Math.toRadians(-40));

    private static final Pose startCollectFirstArtifacts =
            new Pose(48 + ROBOT_LENGTH / 2,
                    (3 * 24) + 5 + (ROBOT_WIDTH/2),
                    Math.toRadians(180));

    private static final Pose endCollectFirstArtifacts =
            new Pose(ROBOT_LENGTH / 2,
                    (3 * 24) + 5 + (ROBOT_WIDTH/2),
                    Math.toRadians(180));

    private static final Pose startCollectSecondArtifacts =
            new Pose(48 + ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 48 + 5,
                    Math.toRadians(180));

    private static final Pose endCollectSecondArtifacts =
            new Pose(ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 48 + 5,
                    Math.toRadians(180));

    /* ================= PATHS ================= */

    private PathChain goShootFirst, goCollectFirst, collectFirst,
            goShootSecond, goCollectSecond, collectSecond, goShootThird, getOut;

    /* ================= SHOOTER ================= */

    private enum ShotPhase { LIFT, DROP, FEED }
    private ShotPhase shotPhase = ShotPhase.LIFT;
    private int shotCount = 0;

    // speed for the motor
    private final double UNIVERSAL_SPEED = 950;

    /* ================= AUTO STATES ================= */

    private enum AutoState {

        GO_SHOOT_FIRST, WAIT_SHOOT_FIRST, SPINUP_FIRST, WAIT_SPINUP_FIRST, SHOOT_FIRST,
        GO_COLLECT_FIRST, WAIT_COLLECT_FIRST, COLLECT_FIRST, WAIT_FINISH_FIRST,

        GO_SHOOT_SECOND, WAIT_SHOOT_SECOND, SPINUP_SECOND, WAIT_SPINUP_SECOND, SHOOT_SECOND,
        GO_COLLECT_SECOND, WAIT_COLLECT_SECOND, COLLECT_SECOND, WAIT_FINISH_SECOND,

        GO_SHOOT_THIRD, WAIT_SHOOT_THIRD, SPINUP_THIRD, WAIT_SPINUP_THIRD, SHOOT_THIRD,

        GET_OUT, WAIT_GET_OUT,

        DONE
    }

    private AutoState autoState = AutoState.GO_SHOOT_FIRST;
    private final ElapsedTime timer = new ElapsedTime();

    /* ================= RUN ================= */

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        buildPaths();
        follower.setStartingPose(startPose);
        swivelTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swivelTurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        intakeMotor.setPower(.9);

        /* hood servo, controls the yellow part.
           the higher the value, the more the hood gets raised.
         */
        hoodServo.setPosition(.76767);

        outtakeMotor.setVelocity(UNIVERSAL_SPEED);

        while (opModeIsActive()) {

            follower.update();

            switch (autoState) {

                /* ========= FIRST CLOSE ========= */

                case GO_SHOOT_FIRST:
                    follower.followPath(goShootFirst);
                    autoState = AutoState.WAIT_SHOOT_FIRST;
                    break;

                case WAIT_SHOOT_FIRST:
                    if (!follower.isBusy())
                        autoState = AutoState.SPINUP_FIRST;
                    break;

                case SPINUP_FIRST:
                    moveMotorToPosition(0, .3);
                    timer.reset();
                    autoState = AutoState.WAIT_SPINUP_FIRST;
                    break;

                case WAIT_SPINUP_FIRST:
                    if (rpmReady(UNIVERSAL_SPEED) && turretReady()) {
                        shotCount = 0;
                        shotPhase = ShotPhase.LIFT;
                        timer.reset();
                        autoState = AutoState.SHOOT_FIRST;
                    }
                    break;

                case SHOOT_FIRST:
                    firstShot(AutoState.GO_COLLECT_FIRST);
                    break;

                /* ========= FIRST ROW ========= */

                case GO_COLLECT_FIRST:
                    follower.followPath(goCollectFirst);
                    autoState = AutoState.WAIT_COLLECT_FIRST;
                    break;

                case WAIT_COLLECT_FIRST:
                    if (!follower.isBusy())
                        autoState = AutoState.COLLECT_FIRST;
                    break;

                case COLLECT_FIRST:
                    follower.followPath(collectFirst);
                    autoState = AutoState.WAIT_FINISH_FIRST;
                    break;

                case WAIT_FINISH_FIRST:
                    if (!follower.isBusy())
                        autoState = AutoState.GO_SHOOT_SECOND;
                    break;

                /* ========= SECOND CLOSE ========= */

                case GO_SHOOT_SECOND:
                    follower.followPath(goShootSecond);
                    autoState = AutoState.WAIT_SHOOT_SECOND;
                    break;

                case WAIT_SHOOT_SECOND:
                    if (!follower.isBusy())
                        autoState = AutoState.SPINUP_SECOND;
                    break;

                case SPINUP_SECOND:
                    moveMotorToPosition(0, .3);
                    timer.reset();
                    autoState = AutoState.WAIT_SPINUP_SECOND;
                    break;

                case WAIT_SPINUP_SECOND:
                    shotCount = 0;
                    shotPhase = ShotPhase.LIFT;
                    timer.reset();
                    autoState = AutoState.SHOOT_SECOND;
                    break;

                case SHOOT_SECOND:
                    runTripleShot(AutoState.GO_COLLECT_SECOND);
                    break;

                /* ========= SECOND ROW ========= */

                case GO_COLLECT_SECOND:
                    follower.followPath(goCollectSecond);
                    autoState = AutoState.WAIT_COLLECT_SECOND;
                    break;

                case WAIT_COLLECT_SECOND:
                    if (!follower.isBusy())
                        autoState = AutoState.COLLECT_SECOND;
                    break;

                case COLLECT_SECOND:
                    follower.followPath(collectSecond);
                    autoState = AutoState.WAIT_FINISH_SECOND;
                    break;

                case WAIT_FINISH_SECOND:
                    if (!follower.isBusy())
                        autoState = AutoState.GO_SHOOT_THIRD;
                    break;

                /* ========= THIRD CLOSE ========= */

                case GO_SHOOT_THIRD:
                    follower.followPath(goShootThird);
                    autoState = AutoState.WAIT_SHOOT_THIRD;
                    break;

                case WAIT_SHOOT_THIRD:
                    if (!follower.isBusy())
                        autoState = AutoState.SPINUP_THIRD;
                    break;

                case SPINUP_THIRD:
                    moveMotorToPosition(0, .3);
                    timer.reset();
                    autoState = AutoState.WAIT_SPINUP_THIRD;
                    break;

                case WAIT_SPINUP_THIRD:
                    shotCount = 0;
                    shotPhase = ShotPhase.LIFT;
                    timer.reset();
                    autoState = AutoState.SHOOT_THIRD;
                    break;

                case SHOOT_THIRD:
                    runTripleShot(AutoState.GET_OUT);
                    break;

                case GET_OUT:
                    follower.followPath(getOut);
                    autoState = AutoState.WAIT_GET_OUT;
                    break;

                case WAIT_GET_OUT:
                    if (!follower.isBusy()) {
                        autoState = AutoState.DONE;
                    }
                    break;

                case DONE:
                    break;
            }

            telemetry.addData("State", autoState);
            telemetry.addData("RPM", outtakeMotor.getVelocity());
            telemetry.update();
        }
    }

    /* ================= SHOOT ROUTINE ================= */

    private void runTripleShot(AutoState nextState) { // actually shoots four times, but only 3 balls

        final double LIFT_TIME = .5;
        final double FEED_TIME = .75;

        if (shotCount >= 4) {
            wipersL.primaryPos();
            wipersR.primaryPos();
            intakeMotor.setPower(.9);
            autoState = nextState;
            return;
        }

        final double REVERSE_POWER = 0;

        switch (shotPhase) {

            case LIFT:
                intakeMotor.setPower(REVERSE_POWER);

                // Alternate: even = right, odd = left
                if (shotCount % 2 == 0) {
                    wipersR.secondaryPos();
                } else {
                    wipersL.secondaryPos();
                }

                timer.reset();
                shotPhase = ShotPhase.DROP;
                break;

            case DROP:
                if (timer.seconds() > LIFT_TIME) {

                    // Return BOTH to primary (safe reset)
                    wipersL.primaryPos();
                    wipersR.primaryPos();

                    intakeMotor.setPower(.9);
                    timer.reset();
                    shotPhase = ShotPhase.FEED;
                }
                break;

            case FEED:
                if (timer.seconds() > FEED_TIME) {
                    intakeMotor.setPower(REVERSE_POWER);
                    shotCount++;
                    shotPhase = ShotPhase.LIFT;
                    timer.reset();
                }
                break;
        }
    }

    private void firstShot(AutoState nextState) { // actually shoots four times, but only 3 balls

        final double LIFT_TIME = .5;
        final double FEED_TIME = .75;

        if (shotCount >= 3) {
            wipersL.primaryPos();
            wipersR.primaryPos();
            intakeMotor.setPower(.9);
            autoState = nextState;
            return;
        }

        final double REVERSE_POWER = 0;

        switch (shotPhase) {

            case LIFT:
                intakeMotor.setPower(REVERSE_POWER);

                if (shotCount % 2 == 0) {
                    wipersL.secondaryPos();
                } else {
                    wipersR.secondaryPos();
                }

                timer.reset();
                shotPhase = ShotPhase.DROP;
                break;

            case DROP:
                if (timer.seconds() > LIFT_TIME) {

                    // Return BOTH to primary (safe reset)
                    wipersL.primaryPos();
                    wipersR.primaryPos();

                    intakeMotor.setPower(.9);
                    timer.reset();
                    shotPhase = ShotPhase.FEED;
                }
                break;

            case FEED:
                if (timer.seconds() > FEED_TIME) {
                    intakeMotor.setPower(REVERSE_POWER);
                    shotCount++;
                    shotPhase = ShotPhase.LIFT;
                    timer.reset();
                }
                break;
        }
    }

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

        goShootFirst = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootClosePos))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootClosePos.getHeading())
                .build();

        goCollectFirst = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePos, startCollectFirstArtifacts))
                .setLinearHeadingInterpolation(shootClosePos.getHeading(), startCollectFirstArtifacts.getHeading())
                .build();

        collectFirst = follower.pathBuilder()
                .addPath(new BezierLine(startCollectFirstArtifacts, endCollectFirstArtifacts))
                .setLinearHeadingInterpolation(startCollectFirstArtifacts.getHeading(), endCollectFirstArtifacts.getHeading())
                .build();

        goShootSecond = follower.pathBuilder()
                .addPath(new BezierLine(endCollectFirstArtifacts, shootClosePos))
                .setLinearHeadingInterpolation(endCollectFirstArtifacts.getHeading(), shootClosePos.getHeading())
                .build();

        goCollectSecond = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePos, startCollectSecondArtifacts))
                .setLinearHeadingInterpolation(shootClosePos.getHeading(), startCollectSecondArtifacts.getHeading())
                .build();

        collectSecond = follower.pathBuilder()
                .addPath(new BezierLine(startCollectSecondArtifacts, endCollectSecondArtifacts))
                .setLinearHeadingInterpolation(startCollectSecondArtifacts.getHeading(), endCollectSecondArtifacts.getHeading())
                .build();

        goShootThird = follower.pathBuilder()
                .addPath(new BezierCurve(
                        endCollectSecondArtifacts,
                        new Pose(24 + ROBOT_LENGTH / 2, 5 + 48 + ROBOT_WIDTH / 2),
                        shootClosePos))
                .setLinearHeadingInterpolation(endCollectSecondArtifacts.getHeading(), shootClosePos.getHeading())
                .build();

        getOut = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePos, endCollectFirstArtifacts))
                .setLinearHeadingInterpolation(shootClosePos.getHeading(), endCollectFirstArtifacts.getHeading())
                .build();
    }

    @Override public void tag20() {}
    @Override public void tag21() {}
    @Override public void tag22() {}
    @Override public void tag23() {}
    @Override public void tag24() {}
    @Override public void nothing() {}
}