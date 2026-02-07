package org.firstinspires.ftc.teamcode.Bluebots;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotFunctions.ChamberState;
import org.firstinspires.ftc.teamcode.RobotFunctions.ColorSensing;
import org.firstinspires.ftc.teamcode.RobotFunctions.Colors;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;

@Autonomous
public class TheDeathOfBlueFarPedroPathing extends TheDeathOfPedroPathing implements LimelightTags {

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
        int id = -1;
        ElapsedTime detectTimer = new ElapsedTime();

        outtakeMotor.setVelocity(2000);
        hoodServo.setPosition(.3);
        swivelTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        moveMotorToPosition(-250, .2);

        while (opModeIsActive()
                && detectTimer.seconds() < 3.0
                && id != 21
                && id != 22
                && id != 23) {

            id = detectTag(limelight, telemetry);
            telemetry.addData("Detected ID", id);
            telemetry.update();
        }

        if (id == 21 || id == 22 || id == 23) {
            motifID = id;
        }

        moveMotorToPosition(-320, .2);

        ElapsedTime spinupTimer = new ElapsedTime();
        while (opModeIsActive()
                && outtakeMotor.getVelocity() < 1990
                && spinupTimer.seconds() < 3.0) {
            telemetry.addLine("Spinning up the outtake...");
            telemetry.addData("Velocity", outtakeMotor.getVelocity());
            telemetry.update();

            idle();
        }

        motifMacroShoot();

        limelight.pipelineSwitch(0);

        while (opModeIsActive()) {
            if (shooting) {
                follower.update();
                continue;
            }
            detectColors();
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
                        moveMotorToPosition(-320, .2);
                        motifMacroShoot();
                        pathState++;
                    }
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
                    break;
            }

            telemetry.addData("Left Chamber", leftState);
            telemetry.addData("Left Stored Color", leftStoredColor);

            telemetry.addData("Right Chamber", rightState);
            telemetry.addData("Right Stored Color", rightStoredColor);
            telemetry.update();
        }
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
