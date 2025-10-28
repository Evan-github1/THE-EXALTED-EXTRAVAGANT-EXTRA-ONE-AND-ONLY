package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;

import java.util.List;

@TeleOp(name = "hello world!")
public class TestNumero3 extends LinearOpMode implements LimelightColor {

    private static Limelight3A limelight;
    private static List<LLResultTypes.FiducialResult> results;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        limelight.pipelineSwitch(1); // green

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            // is able to get x position, just rotate until it's [-5, 5]
            colorDetectionGreen(limelight, telemetry);

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
    public void green() {

    }

    @Override
    public void purple() {

    }
}