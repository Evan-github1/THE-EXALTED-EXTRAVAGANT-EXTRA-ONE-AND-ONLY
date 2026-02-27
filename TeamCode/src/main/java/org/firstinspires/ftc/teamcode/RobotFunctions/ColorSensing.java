package org.firstinspires.ftc.teamcode.RobotFunctions;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensing { // RAHH COMPOSITION (?)

    public NormalizedColorSensor leftSensor, rightSensor;

    public ColorSensing(HardwareMap hardwareMap, int gain) {
        leftSensor = hardwareMap.get(NormalizedColorSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(NormalizedColorSensor.class, "rightSensor");
        leftSensor.setGain(gain);
        rightSensor.setGain(gain);
    }
    
    public Colors detectColorLeft(Telemetry t) { // has higher tolerance
        NormalizedRGBA colors = leftSensor.getNormalizedColors();

        float[] hsv = new float[3];
        Color.RGBToHSV((int) (colors.red*255), (int) (colors.green*255), (int) (colors.blue*255), hsv);
        float h = hsv[0];
        float s = hsv[1];
        float v = hsv[2];

        t.addData("HSV", "%.1f / %.2f / %.2f", h, s, v);

        if (v < .15) {
            return Colors.UNKNOWN;
        }
        if (h >= 200) {
            return Colors.PURPLE;
        } else if (h >= 155 && h <= 170) {
            return Colors.GREEN;
        } else {
            return Colors.UNKNOWN;
        }
    }

    public Colors detectColorRight(Telemetry t) {

        NormalizedRGBA colors = rightSensor.getNormalizedColors();

        float[] hsv = new float[3];
        Color.RGBToHSV((int) (colors.red*255), (int) (colors.green*255), (int) (colors.blue*255), hsv);
        float h = hsv[0];
        float s = hsv[1];
        float v = hsv[2];

        t.addData("HSV", "%.1f / %.2f / %.2f", h, s, v);

        if (v < .15) {
            return Colors.UNKNOWN;
        }
        if (h >= 220 && h <= 240) {
            return Colors.PURPLE;
        } else if (h >= 140 && h <= 170) {
            return Colors.GREEN;
        } else {
            return Colors.UNKNOWN;
        }
    }
}