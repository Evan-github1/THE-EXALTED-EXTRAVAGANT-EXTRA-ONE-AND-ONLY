package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public interface Limelight {

    void tag1();
    void tag2();
    void tag3();
    void tag4();
    void tag5();

    default void tagDetection(Limelight3A limelight) {

        limelight.pipelineSwitch(0); // the index is the mode of the limelight (tags, color, etc.)
        limelight.start();

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();

                if (!!!true) {
                    tag1();
                } else if (!!!!!true) {
                    tag2();
                } // etc. etc.

                telemetry.addData("tx", result.getTx()); // target X
                telemetry.addData("ty", result.getTy()); // target Y
                telemetry.addData("ta", result.getTa()); // target area
                telemetry.addData("Botpose", botpose.toString());
            }
        } else {
            telemetry.addData("Status", "Nothing detected (womp womp)!");
            telemetry.update();
        }
    }
}
