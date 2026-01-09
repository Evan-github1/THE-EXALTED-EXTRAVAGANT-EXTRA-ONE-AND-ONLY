package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensing { // RAHH COMPOSITION (?)

    private static NormalizedColorSensor leftSensor, rightSensor;

    public ColorSensing(HardwareMap hardwareMap, int gain) {
        leftSensor = hardwareMap.get(NormalizedColorSensor.class, "leftSensor");
        rightSensor = hardwareMap.get(NormalizedColorSensor.class, "rightSensor");
        leftSensor.setGain(gain);
        rightSensor.setGain(gain); // 7
    }

    protected static Colors detectColorLeft(Telemetry telemetry) {
        NormalizedRGBA colors = leftSensor.getNormalizedColors();

        float normR, normG, normB;
        normR = colors.red / colors.alpha;
        normG = colors.green / colors.alpha;
        normB = colors.blue / colors.alpha;

        telemetry.addData("Left Red", normR);
        telemetry.addData("Left Green", normG);
        telemetry.addData("Left Blue", normB);

        if (normR < .09 && normG > .25 && normB > .20) {
            return Colors.GREEN;
        } else if (normR > .10 && normG > .15 && normB > .32) {
            return Colors.PURPLE;
        } else {
            return Colors.UNKNOWN;
        }
    }

    protected static Colors detectColorRight(Telemetry telemetry) {
        NormalizedRGBA colors = leftSensor.getNormalizedColors();

        float normR, normG, normB;
        normR = colors.red / colors.alpha;
        normG = colors.green / colors.alpha;
        normB = colors.blue / colors.alpha;

        telemetry.addData("Right Red", normR);
        telemetry.addData("Right Green", normG);
        telemetry.addData("Right Blue", normB);

        if (normR < .09 && normG > .25 && normB > .20) {
            return Colors.GREEN;
        } else if (normR > .10 && normG > .15 && normB > .32) {
            return Colors.PURPLE;
        } else {
            return Colors.UNKNOWN;
        }
    }

    protected enum Colors {
        PURPLE,
        GREEN,
        UNKNOWN;
    }

}