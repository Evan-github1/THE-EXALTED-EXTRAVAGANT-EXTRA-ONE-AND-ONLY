package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
import java.util.List;

public class HuskyLensColorSensing {

    public HuskyLens huskyLens;
    public Colors leftColor, rightColor;

    public HuskyLensColorSensing(HardwareMap hardwareMap) {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        leftColor = Colors.UNKNOWN;
        rightColor = Colors.UNKNOWN;
    }

    public void sort() {
        List<HuskyLens.Block> colors = Arrays.asList(huskyLens.blocks());
        for (HuskyLens.Block color : colors) {

        }
    }
}