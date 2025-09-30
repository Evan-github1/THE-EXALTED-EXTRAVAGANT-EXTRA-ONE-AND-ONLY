package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public interface LimelightTag {

    void tag21();
    void tag22();
    void tag23();
    void nothingDetected();

    default void tagDetection(Limelight3A limelight, Telemetry telemetry) {

        limelight.pipelineSwitch(0); // the index is the mode of the limelight (tags, color, etc.)

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> results;

        if (result.isValid()) {
            results = result.getFiducialResults();
                for (LLResultTypes.FiducialResult r : results) {
                    int id = r.getFiducialId();

                    switch (id) {
                        case 21:
                            tag21();
                            break; // GPP
                        case 22:
                            tag22();
                            break; // PGP
                        case 23:
                            tag23();
                            break; // PPG
                    }
                    telemetry.addData("Detection valid", true);
                    telemetry.addData("AprilTag ID", id);

                }
        }
    }

}

//                Pose3D botpose = result.getBotpose();
//                telemetry.addData("tx", result.getTx());
//                telemetry.addData("ty", result.getTy());
//                telemetry.addData("Botpose", botpose.toString());

