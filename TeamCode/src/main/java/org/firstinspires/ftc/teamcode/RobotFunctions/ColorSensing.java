package org.firstinspires.ftc.teamcode.RobotFunctions;

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

    public Colors detectColorLeft(Telemetry t) {
        NormalizedRGBA colors = leftSensor.getNormalizedColors();

        float r = colors.red;
        float g = colors.green;
        float b = colors.blue;
        float a = colors.alpha;

//        t.addData("R", r);
//        t.addData("G", g);
//        t.addData("B", b);
//        t.addData("Alpha", a);

        if (a < .08) return Colors.UNKNOWN;

        if (a > .9) {
            if (r > .12 && r < .25
                    && g > .17 && g < .27
                    && b > .33 && b < .48) {
                return Colors.PURPLE;
            } else if (r > .05 && r < .12
                    && g > .26 && g < .43
                    && b > .18 && b < .33) {
                return Colors.GREEN;
            }
        } else if (a > .1) {
            if (g > r && g > b) {
                return Colors.GREEN;
            } else if (b > r && b > g) {
                return Colors.PURPLE;
            }
        }
        return Colors.UNKNOWN;
    }

    public Colors detectColorRight(Telemetry t) {
        NormalizedRGBA colors = rightSensor.getNormalizedColors();

        float r = colors.red;
        float g = colors.green;
        float b = colors.blue;
        float a = colors.alpha;

//        t.addData("R", r);
//        t.addData("G", g);
//        t.addData("B", b);
//        t.addData("Alpha", a);

        if (a < .08) return Colors.UNKNOWN;

        if (a > .9) {
            if (r > .12 && r < .20
                    && g > .17 && g < .25
                    && b > .33 && b < .48) {
                return Colors.PURPLE;
            } else if (r > .05 && r < .12
                    && g > .26 && g < .43
                    && b > .18 && b < .33) {
                return Colors.GREEN;
            }
        } else if (a > .1) {
            if (g > r && g > b) {
                return Colors.GREEN;
            } else if (b > r && b > g) {
                return Colors.PURPLE;
            }
        }
        return Colors.UNKNOWN;
    }
}