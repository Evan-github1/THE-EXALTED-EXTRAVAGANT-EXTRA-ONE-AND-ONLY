package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTag;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

import java.net.InetAddress;

@TeleOp
public class AnimosityAndMortification extends LinearOpMode implements LimelightTag, LimelightColor {

    private static Limelight3A limelight;
    private static int timer = 0;
    private static int pipeline = 0; //0 = AprilTag, 1 = Green, 2 = Purple
    private Thread cyclePipelines = new Thread (() -> {
        try {
            tagDetection(limelight, telemetry);
            Thread.sleep(1000);
            colorDetectionGreen(limelight, telemetry);
            Thread.sleep(1000);
            colorDetectionPurple(limelight, telemetry);
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    });

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if (!cyclePipelines.isAlive()) {
                cyclePipelines = new Thread (() -> {
                    try {
                        tagDetection(limelight, telemetry);
                        Thread.sleep(1000);
                        colorDetectionGreen(limelight, telemetry);
                        Thread.sleep(1000);
                        colorDetectionPurple(limelight, telemetry);
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                });
                cyclePipelines.start();
            } 

            telemetry.update();
        }
    }

    public void updatePhoneConsole() {
        telemetry.update();
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
    public void green() {

    }

    @Override
    public void purple() {

    }
}