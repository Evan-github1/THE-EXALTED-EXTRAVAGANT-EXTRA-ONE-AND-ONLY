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
    private static int pipeline = 0; //0 = AprilTag, 1 = Green, 2 = Purple
    private Runnable cyclePipelines = () -> {
            try {
                while(!Thread.currentThread().isInterrupted()) {
                    System.out.println("Thread is running");
                    pipeline = 0;
                    Thread.sleep(1000);
                    pipeline = 1;
                    Thread.sleep(1000);
                    pipeline = 2;
                    Thread.sleep(1000);
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
    };


    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        waitForStart();

        Thread pipelineThread = new Thread(cyclePipelines);
        pipelineThread.start();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            if(pipeline == 0){
                tagDetection(limelight,telemetry);
            }else if(pipeline == 1){
                colorDetectionGreen(limelight,telemetry);
            }else if(pipeline == 2){
                colorDetectionPurple(limelight,telemetry);
            }
            telemetry.update();
        }
        pipelineThread.interrupt();

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