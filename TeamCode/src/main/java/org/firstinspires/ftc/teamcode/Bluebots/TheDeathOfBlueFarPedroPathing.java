package org.firstinspires.ftc.teamcode.Bluebots;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TheDeathOfBlueFarPedroPathing extends EcstasyOfAutomation {

    private static Follower follower;
    private static int pathState;
    private static Pose startPose = new Pose(48 + ROBOT_LENGTH/2, ROBOT_WIDTH/2, Math.toRadians(180));
    private static Pose startCollectFirstArtifcats = new Pose(48 + ROBOT_LENGTH/2, ROBOT_WIDTH/2 + 26, Math.toRadians(180));
    private static Pose endCollectFirstArtifcats = new Pose(ROBOT_LENGTH/2, ROBOT_WIDTH/2 + 26, Math.toRadians(180));

    private static Pose startCollectSecondArtifcats = new Pose(48 + ROBOT_LENGTH/2, ROBOT_WIDTH/2 + 48, Math.toRadians(180));
    private static Pose endCollectSecondArtifcats = new Pose(ROBOT_LENGTH/2, ROBOT_WIDTH/2 + 48, Math.toRadians(180));

    private static Pose shootClosePos = new Pose(48, ROBOT_WIDTH/2 + 96, Math.toRadians(135));

    private static PathChain startToFirstCollect, firstCollect, goBackToStartFromFirstCollect, startToSecondCollect, secondCollect, goToCloseShootPos;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        pathState = 0;
        follower = createFollower(hardwareMap);
        startToFirstCollect = follower.pathBuilder()
            .addPath(new BezierLine(startPose, startCollectFirstArtifcats))
            .setLinearHeadingInterpolation(startPose.getHeading(), startCollectFirstArtifcats.getHeading())
            .build();

        firstCollect = follower.pathBuilder()
                .addPath(new BezierLine(startCollectFirstArtifcats, endCollectFirstArtifcats))
                .setLinearHeadingInterpolation(startCollectFirstArtifcats.getHeading(), endCollectFirstArtifcats.getHeading())
                .build();

        goBackToStartFromFirstCollect = follower.pathBuilder()
                .addPath(new BezierLine(endCollectFirstArtifcats, startPose))
                .setLinearHeadingInterpolation(endCollectFirstArtifcats.getHeading(), startPose.getHeading())
                .build();

        startToSecondCollect = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startCollectSecondArtifcats))
                .setLinearHeadingInterpolation(startPose.getHeading(), startCollectSecondArtifcats.getHeading())
                .build();

        secondCollect = follower.pathBuilder()
                .addPath(new BezierLine(startCollectSecondArtifcats, endCollectSecondArtifcats))
                .setLinearHeadingInterpolation(startCollectSecondArtifcats.getHeading(), endCollectSecondArtifcats.getHeading())
                .build();

        goToCloseShootPos = follower.pathBuilder()
                .addPath(new BezierLine(endCollectSecondArtifcats, shootClosePos))
                .setLinearHeadingInterpolation(endCollectSecondArtifcats.getHeading(), shootClosePos.getHeading())
                .build();

        follower.setStartingPose(startPose);

        waitForStart();

        while(opModeIsActive()) {
            follower.update();
            telemetry.addLine("Running!");

            switch (pathState) {
                case 0:
                    follower.followPath(startToFirstCollect);
                    pathState++;
                    break;
                case 1:
                    if(!follower.isBusy()) {
                        intakeMotor.setPower(1);
                        follower.followPath(firstCollect);
                        pathState++;
                    }
                    break;
                case 2:
                    if(!follower.isBusy()) {
                        intakeMotor.setPower(0);
                        follower.followPath(goBackToStartFromFirstCollect);
                        pathState++;
                    }
                    break;
                case 3:
                    if(!follower.isBusy()) {
                        follower.followPath(startToSecondCollect);
                        pathState++;
                    }
                    break;
                case 4:
                    if(!follower.isBusy()) {
                        follower.followPath(secondCollect);
                        pathState++;
                    }
                    break;
                case 5:
                    if(!follower.isBusy()) {
                        follower.followPath(goToCloseShootPos);
                        pathState++;
                    }
                    break;
            }

            telemetry.update();
        }
    }

    @Override
    public void nothing() {

    }
}
