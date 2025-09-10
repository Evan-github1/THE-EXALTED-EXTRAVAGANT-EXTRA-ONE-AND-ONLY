package org.firstinspires.ftc.teamcode.RobotFunctions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import java.util.Arrays;
import java.util.List;

// TODO: implement this interface to be able to access color detection (HUSKYLENS, NOT LIMELIGHT)
public interface ColorDetection {

    void redDetected();
    void blueDetected();
    void yellowDetected();

    default void colorDetection(HuskyLens huskyLens) {
        List<HuskyLens.Block> blocks = Arrays.asList(huskyLens.blocks());

        if (!blocks.isEmpty()) {
            telemetry.addData("Detected Colors", blocks.size());
            blocks.forEach(block -> {
                /* red ID = 1
                 blue ID = 2
                 yellow ID = 3 */
                        int colorID = block.id;
                        switch (colorID) {
                            case 1:
                                redDetected();
                                telemetry.addData("Red Detected!", "");
                                break;
                            case 2:
                                blueDetected();
                                telemetry.addData("Blue Detected!", "");
                                break;
                            case 3:
                                yellowDetected();
                                telemetry.addData("Yellow Detected!", "");
                                break;
                        }
                        telemetry.addData("Color ID", colorID);
                    });
        } else {
            telemetry.addData("Status", "No Colors Detected");
        }
        telemetry.update();
    }
}
