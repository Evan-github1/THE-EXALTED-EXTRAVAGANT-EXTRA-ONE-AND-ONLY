package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public interface LimelightColor {
    void green();
    void purple();

    default void colorDetectionGreen(Limelight3A limelight, Telemetry telemetry) {

        limelight.pipelineSwitch(1); // green
        LLResult resultGreen = limelight.getLatestResult();
        if (resultGreen != null) {
            if (resultGreen.isValid()) {
                if (!resultGreen.getColorResults().isEmpty()) {
                    Pose3D botpose = resultGreen.getBotpose();
                    telemetry.addData("tx", resultGreen.getTx());
                    telemetry.addData("ty", resultGreen.getTy());
                    telemetry.addData("Botpose", botpose.toString());

                    green();
                    telemetry.addData("Green Detected", true);
                }
            }
        }
    }

    default void colorDetectionPurple(Limelight3A limelight, Telemetry telemetry) {
        limelight.pipelineSwitch(2); // purple
        LLResult resultPurple = limelight.getLatestResult();
        if (resultPurple != null) {
            if (resultPurple.isValid()) {
                if (!resultPurple.getColorResults().isEmpty()) {
                    purple();
                    telemetry.addData("Purple Detected", true);
                }
            }
        }

        telemetry.update();

    }
}