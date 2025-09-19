package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public interface LimelightTag {

    void tag21();
    void tag22();
    void tag23();

    default boolean tagDetection(Limelight3A limelight, Telemetry telemetry) {

        limelight.pipelineSwitch(0);

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
//                Pose3D botpose = result.getBotpose();
//                telemetry.addData("tx", result.getTx());
//                telemetry.addData("ty", result.getTy());
//                telemetry.addData("Botpose", botpose.toString());

                List<LLResultTypes.FiducialResult> results = result.getFiducialResults();
                for (LLResultTypes.FiducialResult r : results) {
                    int id = r.getFiducialId();
                    telemetry.addData("AprilTag ID", id);
                    switch (id) {
                        case 21: // GPP
                            tag21();
                            return true;
                        case 22: // PGP
                            tag22();
                            return true;
                        case 23: // PPG
                            tag23();
                            return true;
                    }
                }
            }
        } else {
            telemetry.addData("Status", "Nothing detected (womp womp)!");
        }
        telemetry.update();
        return false;
    }

}
