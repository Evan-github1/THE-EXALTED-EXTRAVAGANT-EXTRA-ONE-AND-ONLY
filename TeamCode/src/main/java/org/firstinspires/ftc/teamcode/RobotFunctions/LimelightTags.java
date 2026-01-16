package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public interface LimelightTags {

    default int detectTag(Limelight3A limelight, Telemetry telemetry) {
        List<LLResultTypes.FiducialResult> results;
        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            results = result.getFiducialResults();
            for (LLResultTypes.FiducialResult r : results) {
                int id = r.getFiducialId();
                telemetry.addData("AprilTag ID",id);
                switch (id) {
                    case 20:
                        tag20();
                        return 20;
                    case 21:
                        tag21();
                        return 21; // GPP
                    case 22:
                        tag22();
                        return 22; // PGP
                    case 23:
                        tag23();
                        return 23; // PPG
                    case 24:
                        tag24();
                        return 24;
                }
            }
        }

        return -1;
    }

    default LLResultTypes.FiducialResult detectTagSelective(Limelight3A limelight, Telemetry telemetry) {
        List<LLResultTypes.FiducialResult> results;
        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            results = result.getFiducialResults();
            LLResultTypes.FiducialResult targeted = null;
            double tx = Double.MAX_VALUE;
            for (LLResultTypes.FiducialResult r : results) {
                int id = r.getFiducialId();
                if(id == 20 || id == 24){
                    telemetry.addData("AprilTag ID",id);
                    if(r.getTargetXDegrees() < tx){
                        targeted = r;
                        tx = r.getTargetXDegrees();
                    }
                }
            }
            return targeted;
        }
        return null;
    }

    default double getTA(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            return result.getTa();
        }
        return -1;
    }

    default double getTX(Limelight3A limelight){
        LLResult result = limelight.getLatestResult();
        if(result.isValid()){
            return result.getTx();
        }
        return Double.NaN;
    }

    default double getDistanceFromTag(double ta) { // ta = target area
        double scale = 30665.95;
        double distance = (scale/ ta);
        return distance;
    } //Doesn't work btw



    void tag20();
    void tag21();
    void tag22();
    void tag23();
    void tag24();
}
