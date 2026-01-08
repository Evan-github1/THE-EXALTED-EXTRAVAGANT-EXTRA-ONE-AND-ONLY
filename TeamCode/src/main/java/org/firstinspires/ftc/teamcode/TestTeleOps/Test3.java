package org.firstinspires.ftc.teamcode.TestTeleOps;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;

import java.util.List;

@TeleOp
public class Test3 extends LinearOpMode implements LimelightTags {

    private static Limelight3A limelight;
    private static List<LLResultTypes.FiducialResult> results;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        limelight.pipelineSwitch(0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            telemetry.addData("ID", detectTag(limelight, telemetry));

            telemetry.update();

        }
    }

    public boolean delay() {
        return System.currentTimeMillis() >= time + 250;
    }

    public boolean delay(long duration){
        return System.currentTimeMillis() >= time + duration;
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