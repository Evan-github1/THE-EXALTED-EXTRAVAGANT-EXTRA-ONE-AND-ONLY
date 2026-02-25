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
public class TheDeathOfPedroPathing_BLUE_CLOSE extends TheDeathOfPedroPathing implements LimelightTags {

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
                    (3 * 24) + 12 + (ROBOT_WIDTH/2),
                    Math.toRadians(180));

    private static final Pose endCollectFirstArtifacts =
            new Pose(ROBOT_LENGTH / 2,
                    (3 * 24) + 12 + (ROBOT_WIDTH/2),
                    Math.toRadians(180));

    private static final Pose startCollectSecondArtifacts =
            new Pose(48 + ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 48,
                    Math.toRadians(180));

    private static final Pose endCollectSecondArtifacts =
            new Pose(ROBOT_LENGTH / 2,
                    ROBOT_WIDTH / 2 + 48,
                    Math.toRadians(180));

    /* ================= PATHS ================= */

    private PathChain goShootFirst, goCollectFirst, collectFirst, goShootSecond,
    goCollectSecond, collectSecond, goShootThird;

    /* ================= SHOOTER ================= */
    private enum ShotPhase { LIFT, DROP, FEED }
    private ShotPhase shotPhase = ShotPhase.LIFT;
    private int shotCount = 0;
    /* ================= AUTO STATES ================= */

    private enum AutoState {

    }

    private AutoState autoState;
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

            }

            telemetry.addData("State", autoState);
            telemetry.addData("Shots", shotCount);
            telemetry.addData("RPM", outtakeMotor.getVelocity());
            telemetry.update();
        }
    }

    /* ================= SHOOT ROUTINES ================= */

    private void runTripleShot(AutoState nextState) {

        final double LIFT_TIME = 0.9;
        final double FEED_TIME = 0.5;

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
                .addPath(new BezierLine(endCollectSecondArtifacts, shootClosePos))
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
