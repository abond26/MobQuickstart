package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.dumbMap;

public class ColorTesting {
    NormalizedColorSensor colorSensor;
    Servo intakeShifterR;
    Servo intakeShifterL;
    
    // Track balllllll
    ElapsedTime colorTimer = new ElapsedTime();
    DetectedColor currentlyDetectedColor = DetectedColor.UNKNOWN;
    //how long should i hold this b all
    public double requiredDetectionTimeSeconds = 3;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        
        intakeShifterR = hardwareMap.get(Servo.class, "intakeShifterR");
        intakeShifterR.setDirection(Servo.Direction.FORWARD);
        intakeShifterL = hardwareMap.get(Servo.class, "intakeShifterL");
        intakeShifterL.setDirection(Servo.Direction.REVERSE);
        
        colorTimer.reset();
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        
        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);
        
        DetectedColor detectedThisFrame = DetectedColor.UNKNOWN;
        
        if (normRed > 0.3 && normBlue > 0.3 && normGreen < 0.2) {
            detectedThisFrame = DetectedColor.PURPLE;
        } else if (normGreen > 0.3 && normRed < 0.2 && normBlue < 0.2) {
            detectedThisFrame = DetectedColor.GREEN;
        }
        
        if (detectedThisFrame != currentlyDetectedColor) {
            currentlyDetectedColor = detectedThisFrame;
            colorTimer.reset();
        }

        telemetry.addData("What it hink da color is", currentlyDetectedColor);
        telemetry.addData("Time Detected in secondssssssss", colorTimer.seconds());

        if (currentlyDetectedColor != DetectedColor.UNKNOWN && colorTimer.seconds() >= requiredDetectionTimeSeconds) {
            
            if (currentlyDetectedColor == DetectedColor.PURPLE || currentlyDetectedColor == DetectedColor.GREEN) {
                intakeShifterR.setPosition(0.0);
                intakeShifterL.setPosition(0.0);
            }
            
            return currentlyDetectedColor;
        } else {
            intakeShifterR.setPosition(0.54);
            intakeShifterL.setPosition(0.54);
        }

        return DetectedColor.UNKNOWN;

    }
}
