package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dumbMap;

public class ColorTesting {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red/ colors.alpha;
        normGreen= colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;
        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
        ////TODO add if statements for specific colors

        return DetectedColor.UNKNOWN;

    }
}
