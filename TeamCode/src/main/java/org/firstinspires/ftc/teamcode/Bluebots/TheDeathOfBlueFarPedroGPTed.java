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
public class TheDeathOfBlueFarPedroGPTed extends TheDeathOfPedroPathing
        implements LimelightTags {

    // ---------------- POSES ----------------

    private static final Pose startPose =
            new Pose(48 + ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2,
                    Math.toRadians(180));

    private static final Pose startCollectFirstArtifacts =
            new Pose(48 + ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 26,
                    Math.toRadians(180));

    private static final Pose endCollectFirstArtifacts =
            new Pose(ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 26,
                    Math.toRadians(180));

    private static final Pose resetStartPose =
            new Pose(48 + ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 2,
                    Math.toRadians(180));

    private static final Pose startCollectSecondArtifacts =
            new Pose(48 + ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 48,
                    Math.toRadians(180));

    private static final Pose endCollectSecondArtifacts =
            new Pose(ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 48,
                    Math.toRadians(180));

    private static final Pose shootClosePos =
            new Pose(48,
                    ROBOT_WIDTH / 2 + 96,
                    Math.toRadians(-45));

    // ---------------- PATHS ----------------

    private PathChain startToFirstCollect;
    private PathChain firstCollect;
    private PathChain goBackToStartFromFirstCollect;
    private PathChain startToSecondCollect;
    private PathChain secondCollect;
    private PathChain goToCloseShootPos;

    // ---------------- STATE ----------------

    private enum AutoState {

        PRELOAD_SPINUP,
        PRELOAD_WAIT_RPM,
        PRELOAD_WAIT_TURRET,
        PRELOAD_FIRE,
        PRELOAD_WAIT_WIPER,

        PATH_TO_FIRST,
        WAIT_FIRST,
        COLLECT_FIRST,

        PATH_BACK_1,
        WAIT_BACK_1,

        MID_FIRE,
        MID_WAIT_WIPER,

        PATH_TO_SECOND,
        WAIT_SECOND,
        COLLECT_SECOND,

        PATH_TO_CLOSE,
        WAIT_CLOSE,

        CLOSE_FIRE,
        CLOSE_WAIT_WIPER,

        DONE
    }

    private AutoState autoState = AutoState.PRELOAD_SPINUP;
    private final ElapsedTime timer = new ElapsedTime();

    // ---------------- RUN ----------------

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

                // -------- PRELOAD --------

                case PRELOAD_SPINUP:
                    outtakeMotor.setVelocity(1900);
                    hoodServo.setPosition(.2);
                    moveMotorToPosition(-315, .2);

                    intakeMotor.setPower(1);

                    timer.reset();
                    autoState = AutoState.PRELOAD_WAIT_RPM;
                    break;

                case PRELOAD_WAIT_RPM:
                    if (Math.abs(outtakeMotor.getVelocity() - 1900) < 75
                            || timer.seconds() > 2.5) {

                        timer.reset();
                        autoState = AutoState.PRELOAD_WAIT_TURRET;
                    }
                    break;

                case PRELOAD_WAIT_TURRET:
                    if (!swivelTurretMotor.isBusy()
                            || timer.seconds() > 4) {

                        autoState = AutoState.PRELOAD_FIRE;
                    }
                    break;

                case PRELOAD_FIRE:
                    intakeMotor.setPower(0);
                    wipersR.secondaryPos();

                    timer.reset();
                    autoState = AutoState.PRELOAD_WAIT_WIPER;
                    break;

                case PRELOAD_WAIT_WIPER:
                    if (timer.seconds() > 0.5) {

                        wipersR.primaryPos();
                        intakeMotor.setPower(1);

                        autoState = AutoState.PATH_TO_FIRST;
                    }
                    break;

                // -------- FIRST COLLECT --------

                case PATH_TO_FIRST:
                    follower.followPath(startToFirstCollect);
                    autoState = AutoState.WAIT_FIRST;
                    break;

                case WAIT_FIRST:
                    if (!follower.isBusy())
                        autoState = AutoState.COLLECT_FIRST;
                    break;

                case COLLECT_FIRST:
                    follower.followPath(firstCollect, .5, true);
                    autoState = AutoState.PATH_BACK_1;
                    break;

                case PATH_BACK_1:
                    if (!follower.isBusy()) {
                        follower.followPath(goBackToStartFromFirstCollect);
                        autoState = AutoState.WAIT_BACK_1;
                    }
                    break;

                case WAIT_BACK_1:
                    if (!follower.isBusy())
                        autoState = AutoState.MID_FIRE;
                    break;

                // -------- MID SHOT --------

                case MID_FIRE:
                    outtakeMotor.setVelocity(1900);
                    hoodServo.setPosition(.2);
                    moveMotorToPosition(-315, .2);

                    intakeMotor.setPower(0);
                    wipersR.secondaryPos();

                    timer.reset();
                    autoState = AutoState.MID_WAIT_WIPER;
                    break;

                case MID_WAIT_WIPER:
                    if (timer.seconds() > 0.5) {

                        wipersR.primaryPos();
                        intakeMotor.setPower(1);

                        autoState = AutoState.PATH_TO_SECOND;
                    }
                    break;

                // -------- SECOND COLLECT --------

                case PATH_TO_SECOND:
                    follower.followPath(startToSecondCollect);
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

                // -------- CLOSE SHOT --------

                case PATH_TO_CLOSE:
                    if (!follower.isBusy()) {
                        follower.followPath(goToCloseShootPos);
                        autoState = AutoState.WAIT_CLOSE;
                    }
                    break;

                case WAIT_CLOSE:
                    if (!follower.isBusy())
                        autoState = AutoState.CLOSE_FIRE;
                    break;

                case CLOSE_FIRE:
                    outtakeMotor.setVelocity(1500);
                    hoodServo.setPosition(.5);
                    moveMotorToPosition(0, .3);

                    intakeMotor.setPower(0);
                    wipersR.secondaryPos();

                    timer.reset();
                    autoState = AutoState.CLOSE_WAIT_WIPER;
                    break;

                case CLOSE_WAIT_WIPER:
                    if (timer.seconds() > 0.5) {

                        wipersR.primaryPos();
                        intakeMotor.setPower(1);

                        autoState = AutoState.DONE;
                    }
                    break;

                case DONE:
                    follower.breakFollowing();
                    break;
            }

            telemetry.addData("State", autoState);
            telemetry.addData("RPM", outtakeMotor.getVelocity());
            telemetry.addData("Intake", intakeMotor.getPower());
            telemetry.update();
        }
    }

    // ---------------- PATH BUILDER ----------------

    private void buildPaths() {

        startToFirstCollect = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startCollectFirstArtifacts))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        startCollectFirstArtifacts.getHeading())
                .build();

        firstCollect = follower.pathBuilder()
                .addPath(new BezierLine(
                        startCollectFirstArtifacts,
                        endCollectFirstArtifacts))
                .setLinearHeadingInterpolation(
                        startCollectFirstArtifacts.getHeading(),
                        endCollectFirstArtifacts.getHeading())
                .build();

        goBackToStartFromFirstCollect = follower.pathBuilder()
                .addPath(new BezierLine(
                        endCollectFirstArtifacts,
                        resetStartPose))
                .setLinearHeadingInterpolation(
                        endCollectFirstArtifacts.getHeading(),
                        resetStartPose.getHeading())
                .build();

        startToSecondCollect = follower.pathBuilder()
                .addPath(new BezierLine(
                        resetStartPose,
                        startCollectSecondArtifacts))
                .setLinearHeadingInterpolation(
                        resetStartPose.getHeading(),
                        startCollectSecondArtifacts.getHeading())
                .build();

        secondCollect = follower.pathBuilder()
                .addPath(new BezierLine(
                        startCollectSecondArtifacts,
                        endCollectSecondArtifacts))
                .setLinearHeadingInterpolation(
                        startCollectSecondArtifacts.getHeading(),
                        endCollectSecondArtifacts.getHeading())
                .build();

        goToCloseShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(
                        endCollectSecondArtifacts,
                        new Pose(
                                24 + ROBOT_LENGTH / 2,
                                48 + ROBOT_WIDTH / 2),
                        shootClosePos))
                .setLinearHeadingInterpolation(
                        endCollectSecondArtifacts.getHeading(),
                        shootClosePos.getHeading())
                .build();
    }

    // ---------------- TURRET ----------------

    private void moveMotorToPosition(int targetPosition, double power) {

        swivelTurretMotor.setTargetPosition(targetPosition);
        swivelTurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swivelTurretMotor.setPower(power);
    }

    // ---------------- LIMELIGHT TAGS ----------------

    @Override public void tag20() {}
    @Override public void tag21() {}
    @Override public void tag22() {}
    @Override public void tag23() {}
    @Override public void tag24() {}
    @Override public void nothing() {}
}
