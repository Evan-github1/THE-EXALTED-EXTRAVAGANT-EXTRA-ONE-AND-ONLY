package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface LimelightColor {
    void green();
    void purple();

    default boolean colorDetectionGreen(Limelight3A limelight, Telemetry telemetry) {
        limelight.pipelineSwitch(1); // green
        LLResult resultGreen = limelight.getLatestResult();
        if (resultGreen != null) {
            if (resultGreen.isValid()) {
                if (!resultGreen.getColorResults().isEmpty()) {
                    green();
                    telemetry.addData("Green Detected", true);
                    return true;
                }
            }
        }
        telemetry.update();
        return false;
    }

    default boolean colorDetectionPurple(Limelight3A limelight, Telemetry telemetry) {
        limelight.pipelineSwitch(2); // purple
        LLResult resultPurple = limelight.getLatestResult();
        if (resultPurple != null) {
            if (resultPurple.isValid()) {
                if (!resultPurple.getColorResults().isEmpty()) {
                    purple();
                    telemetry.addData("Purple Detected", true);
                    return true;
                }
            }
        }

        telemetry.update();
        return false;
    }
}
