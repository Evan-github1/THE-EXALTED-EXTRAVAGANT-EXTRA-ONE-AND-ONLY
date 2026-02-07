package org.firstinspires.ftc.teamcode.Bluebots;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;

@Disabled
@Autonomous
public class TheDeathOfBlueFarCrappyPedroPathing extends TheDeathOfPedroPathing implements LimelightTags {

    private static Pose startPose = new Pose(48 + ROBOT_LENGTH/2, ROBOT_WIDTH/2, Math.toRadians(180));
    private static Pose startCollectFirstArtifcats = new Pose(48 + ROBOT_LENGTH/2, ROBOT_WIDTH/2 + 26, Math.toRadians(180));
    private static Pose endCollectFirstArtifcats = new Pose(ROBOT_LENGTH/2, ROBOT_WIDTH/2 + 26, Math.toRadians(180));

    private static Pose resetStartPose = new Pose(48 + ROBOT_LENGTH/2, ROBOT_WIDTH/2 + 2, Math.toRadians(180));
    private static Pose startCollectSecondArtifcats = new Pose(48 + ROBOT_LENGTH/2, ROBOT_WIDTH/2 + 48, Math.toRadians(180));
    private static Pose endCollectSecondArtifcats = new Pose(ROBOT_LENGTH/2, ROBOT_WIDTH/2 + 48, Math.toRadians(180));

    private static Pose shootClosePos = new Pose(48, ROBOT_WIDTH/2 + 96, Math.toRadians(-45));

    private static PathChain startToFirstCollect, firstCollect, goBackToStartFromFirstCollect, startToSecondCollect, secondCollect, goToCloseShootPos;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        startToFirstCollect = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startCollectFirstArtifcats))
                .setLinearHeadingInterpolation(startPose.getHeading(), startCollectFirstArtifcats.getHeading())
                .build();

        firstCollect = follower.pathBuilder()
                .addPath(new BezierLine(startCollectFirstArtifcats, endCollectFirstArtifcats))
                .setLinearHeadingInterpolation(startCollectFirstArtifcats.getHeading(), endCollectFirstArtifcats.getHeading())
                .build();

        goBackToStartFromFirstCollect = follower.pathBuilder()
                .addPath(new BezierLine(endCollectFirstArtifcats, resetStartPose))
                .setLinearHeadingInterpolation(endCollectFirstArtifcats.getHeading(), resetStartPose.getHeading())
                .build();

        startToSecondCollect = follower.pathBuilder()
                .addPath(new BezierLine(resetStartPose, startCollectSecondArtifcats))
                .setLinearHeadingInterpolation(resetStartPose.getHeading(), startCollectSecondArtifcats.getHeading())
                .build();

        secondCollect = follower.pathBuilder()
                .addPath(new BezierLine(startCollectSecondArtifcats, endCollectSecondArtifcats))
                .setLinearHeadingInterpolation(startCollectSecondArtifcats.getHeading(), endCollectSecondArtifcats.getHeading())
                .build();

        goToCloseShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(endCollectSecondArtifcats, new Pose(24 + ROBOT_LENGTH/2, 48 + ROBOT_WIDTH/2), shootClosePos))
                .setLinearHeadingInterpolation(endCollectSecondArtifcats.getHeading(), shootClosePos.getHeading())
                .build();

        follower.setStartingPose(startPose);

        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        intakeMotor.setPower(1);
        outtakeMotor.setVelocity(2000);
        swivelTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shootFromFar();

        limelight.pipelineSwitch(0);

        while (opModeIsActive()) {
            follower.update();

            telemetry.addLine("Running!");
            switch (pathState) {
                case 0:
                    follower.followPath(startToFirstCollect);
                    pathState++;
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(firstCollect, .5, true);
                        pathState++;
                    }
                    break;

                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(goBackToStartFromFirstCollect);
                        pathState++;
                    }
                    shootFromFar();
                    break;

                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(startToSecondCollect);
                        pathState++;
                    }
                    break;

                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(secondCollect, .5, true);
                        pathState++;
                    }
                    break;

                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(goToCloseShootPos);
                        pathState++;
                    }
                    shootFromClose();
                    break;
            }
            telemetry.update();
        }
    }

    private void shootFromFar() {
        final double V = 1700;
        outtakeMotor.setVelocity(V);
        hoodServo.setPosition(.2);

        ElapsedTime spinupTimer = new ElapsedTime();

        while (opModeIsActive()
                && (outtakeMotor.getVelocity() <  V - 50
                || outtakeMotor.getVelocity() > V + 50)
                && spinupTimer.seconds() < 6.0) {
            telemetry.addLine("Spinning up the outtake...");
            telemetry.addData("Velocity", outtakeMotor.getVelocity());
            telemetry.update();

            idle();
        }

        moveMotorToPosition(-320, .2);

        ElapsedTime turnTimer = new ElapsedTime();

        while (opModeIsActive()
                && swivelTurretMotor.getCurrentPosition() < -340
                && swivelTurretMotor.getCurrentPosition() > -300
                && turnTimer.seconds() < 4.0) {
            telemetry.addLine("Turning the turret...");
            telemetry.addData("Position", swivelTurretMotor);
            telemetry.update();

            idle();
        }

        liftLeftWiperNT();
        liftRightWiperNT();
        liftLeftWiperNT();
    }

    private void shootFromClose() {
        final double V = 1500;
        outtakeMotor.setVelocity(V);
        hoodServo.setPosition(.5);

        ElapsedTime spinupTimer = new ElapsedTime();

        while (opModeIsActive()
                && (outtakeMotor.getVelocity() < V - 50
                || outtakeMotor.getVelocity() > V + 50)
                && spinupTimer.seconds() < 6.0) {
            telemetry.addLine("Slowing down the outtake...");
            telemetry.addData("Velocity", outtakeMotor.getVelocity());
            telemetry.update();

            idle();
        }

        moveMotorToPosition(0, .2);

        ElapsedTime turnTimer = new ElapsedTime();

        while (opModeIsActive()
                && swivelTurretMotor.getCurrentPosition() < -20
                && swivelTurretMotor.getCurrentPosition() > 20
                && turnTimer.seconds() < 4.0) {
            telemetry.addLine("Turning the turret...");
            telemetry.addData("Position", swivelTurretMotor);
            telemetry.update();

            idle();
        }
        liftLeftWiperNT();
        liftRightWiperNT();
        liftLeftWiperNT();
    }

    @Override
    protected void liftRightWiperNT() {
        intakeMotor.setPower(0);
        wipersR.secondaryPos();
        sleep(500);
        wipersR.primaryPos();
        intakeMotor.setPower(1);
    }

    @Override
    protected void liftLeftWiperNT() {
        intakeMotor.setPower(0);
        wipersL.secondaryPos();
        sleep(500);
        wipersL.primaryPos();
        intakeMotor.setPower(1);
    }

    public void moveMotorToPosition(int targetPosition, double power) {
        swivelTurretMotor.setTargetPosition(targetPosition);
        swivelTurretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        swivelTurretMotor.setPower(power);
    }


    @Override
    public void tag20() {}

    @Override
    public void tag21() {}

    @Override
    public void tag22() {}

    @Override
    public void tag23() {}

    @Override
    public void tag24() {}

    @Override
    public void nothing() {}
}
