package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

public interface LimelightTag {

    void tag21();
    void tag22();
    void tag23();

    default void tagDetection(Limelight3A limelight, Telemetry telemetry) {

        limelight.pipelineSwitch(0); // the index is the mode of the limelight (tags, color, etc.)

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
//                  Pose3D botpose = result.getBotpose();
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
                            break;
                        case 22: // PGP
                            tag22();
                            break;
                        case 23: // PPG
                            tag23();
                            break;
                        default: break;
                    }
                }
            }
        } else {
            telemetry.addData("Status", "Nothing detected (womp womp)!");
        }
        telemetry.update();

    }

}
