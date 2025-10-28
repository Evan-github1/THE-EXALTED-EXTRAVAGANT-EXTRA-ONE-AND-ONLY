package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface LimelightColor {
    void green();
    void purple();

    default void colorDetectionGreen(Limelight3A limelight, Telemetry telemetry) { // pipeline = 1
        LLResult resultGreen = limelight.getLatestResult();
        if (resultGreen != null && resultGreen.isValid()) {
            green();
            telemetry.addData("Green",
                    "Detected: %s | tx: %.2f | ty: %.2f",
                    resultGreen.isValid(), resultGreen.getTx(), resultGreen.getTy());
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