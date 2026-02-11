package org.firstinspires.ftc.teamcode.TestTeleOps;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;

@TeleOp
public class Test3 extends LinearOpMode implements LimelightTags {

    private static Limelight3A limelight;
    private static double distance;
    private static double delta_mount;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        limelight.pipelineSwitch(0);

        delta_mount = 19;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            double height_target, height_camera, delta_target;

            height_target = 29.25;
            height_camera = 13.75;
            delta_target = getTY(limelight);

            if (gamepad1.dpadUpWasPressed()) {
                delta_mount += .1;
            } else if (gamepad1.dpadDownWasPressed()) {
                delta_mount -= .1;
            }
            distance = (height_target - height_camera)/(Math.tan(Math.toRadians(delta_mount + delta_target)));

            telemetry.addData("Distance", distance);
            telemetry.addData("Delta Mount", delta_mount);
            telemetry.update();
        }
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

    @Override
    public void nothing() {

    }
}