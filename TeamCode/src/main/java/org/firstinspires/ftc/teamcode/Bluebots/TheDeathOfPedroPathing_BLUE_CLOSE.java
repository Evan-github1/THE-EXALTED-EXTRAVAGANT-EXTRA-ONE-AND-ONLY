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

    private static final Pose shootClosePos =
            new Pose(48,
                    ROBOT_WIDTH / 2 + 96,
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
            goShootSecond, goCollectSecond, collectSecond, goShootThird;

    /* ================= AUTO STATES ================= */

    private enum AutoState {

        GO_SHOOT_FIRST,
        WAIT_SHOOT_FIRST,

        GO_COLLECT_FIRST,
        WAIT_COLLECT_FIRST,
        COLLECT_FIRST,
        WAIT_FINISH_FIRST,

        GO_SHOOT_SECOND,
        WAIT_SHOOT_SECOND,

        GO_COLLECT_SECOND,
        WAIT_COLLECT_SECOND,
        COLLECT_SECOND,
        WAIT_FINISH_SECOND,

        GO_SHOOT_THIRD,
        WAIT_SHOOT_THIRD,

        DONE
    }

    private AutoState autoState = AutoState.GO_SHOOT_FIRST;

    /* ================= RUN ================= */

    @Override
    public void runOpMode() throws InterruptedException {

        super.runOpMode();
        buildPaths();
        follower.setStartingPose(startPose);

        waitForStart();

        intakeMotor.setPower(1);
        hoodServo.setPosition(.5);

        while (opModeIsActive()) {

            follower.update();

            switch (autoState) {

                case GO_SHOOT_FIRST:
                    follower.followPath(goShootFirst);
                    autoState = AutoState.WAIT_SHOOT_FIRST;
                    break;

                case WAIT_SHOOT_FIRST:
                    if (!follower.isBusy())
                        autoState = AutoState.GO_COLLECT_FIRST;
                    break;

                case GO_COLLECT_FIRST:
                    follower.followPath(goCollectFirst);
                    autoState = AutoState.WAIT_COLLECT_FIRST;
                    break;

                case WAIT_COLLECT_FIRST:
                    if (!follower.isBusy())
                        autoState = AutoState.COLLECT_FIRST;
                    break;

                case COLLECT_FIRST:
                    follower.followPath(collectFirst, .75, true);
                    autoState = AutoState.WAIT_FINISH_FIRST;
                    break;

                case WAIT_FINISH_FIRST:
                    if (!follower.isBusy())
                        autoState = AutoState.GO_SHOOT_SECOND;
                    break;

                case GO_SHOOT_SECOND:
                    follower.followPath(goShootSecond);
                    autoState = AutoState.WAIT_SHOOT_SECOND;
                    break;

                case WAIT_SHOOT_SECOND:
                    if (!follower.isBusy())
                        autoState = AutoState.GO_COLLECT_SECOND;
                    break;

                case GO_COLLECT_SECOND:
                    follower.followPath(goCollectSecond);
                    autoState = AutoState.WAIT_COLLECT_SECOND;
                    break;

                case WAIT_COLLECT_SECOND:
                    if (!follower.isBusy())
                        autoState = AutoState.COLLECT_SECOND;
                    break;

                case COLLECT_SECOND:
                    follower.followPath(collectSecond, .75, true);
                    autoState = AutoState.WAIT_FINISH_SECOND;
                    break;

                case WAIT_FINISH_SECOND:
                    if (!follower.isBusy())
                        autoState = AutoState.GO_SHOOT_THIRD;
                    break;

                case GO_SHOOT_THIRD:
                    follower.followPath(goShootThird);
                    autoState = AutoState.WAIT_SHOOT_THIRD;
                    break;

                case WAIT_SHOOT_THIRD:
                    if (!follower.isBusy())
                        autoState = AutoState.DONE;
                    break;

                case DONE:
                    follower.breakFollowing();
                    break;
            }

            telemetry.addData("State", autoState);
            telemetry.update();
        }
    }

    /* ================= PATHS ================= */

    private void buildPaths() {

        goShootFirst = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootClosePos))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        shootClosePos.getHeading())
                .build();

        goCollectFirst = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePos, startCollectFirstArtifacts))
                .setLinearHeadingInterpolation(
                        shootClosePos.getHeading(),
                        startCollectFirstArtifacts.getHeading())
                .build();

        collectFirst = follower.pathBuilder()
                .addPath(new BezierLine(startCollectFirstArtifacts, endCollectFirstArtifacts))
                .setLinearHeadingInterpolation(
                        startCollectFirstArtifacts.getHeading(),
                        endCollectFirstArtifacts.getHeading())
                .build();

        goShootSecond = follower.pathBuilder()
                .addPath(new BezierLine(endCollectFirstArtifacts, shootClosePos))
                .setLinearHeadingInterpolation(
                        endCollectFirstArtifacts.getHeading(),
                        shootClosePos.getHeading())
                .build();

        goCollectSecond = follower.pathBuilder()
                .addPath(new BezierLine(shootClosePos, startCollectSecondArtifacts))
                .setLinearHeadingInterpolation(
                        shootClosePos.getHeading(),
                        startCollectSecondArtifacts.getHeading())
                .build();

        collectSecond = follower.pathBuilder()
                .addPath(new BezierLine(startCollectSecondArtifacts, endCollectSecondArtifacts))
                .setLinearHeadingInterpolation(
                        startCollectSecondArtifacts.getHeading(),
                        endCollectSecondArtifacts.getHeading())
                .build();

        goShootThird = follower.pathBuilder()
                .addPath(new BezierCurve(
                        endCollectSecondArtifacts,
                        new Pose(24 + ROBOT_LENGTH / 2, 5 + 48 + ROBOT_WIDTH / 2),
                        shootClosePos))
                .setLinearHeadingInterpolation(
                        endCollectSecondArtifacts.getHeading(),
                        shootClosePos.getHeading())
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