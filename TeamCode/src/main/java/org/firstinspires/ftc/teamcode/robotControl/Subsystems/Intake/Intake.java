package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements IntakeConstants{
    private DcMotor intakeL, intakeR;
    private Servo intakeShifterR, intakeShifterL;
    private NormalizedColorSensor colorSensor;

    // Color detection state
    private ElapsedTime colorTimer = new ElapsedTime();
    private DetectedColor currentlyDetectedColor = DetectedColor.UNKNOWN;
    public double requiredDetectionTimeSeconds = 0.3;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    public Intake(@NonNull HardwareMap hardwareMap){
        intakeL = hardwareMap.get(DcMotor.class, "intakeL");
        intakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setPower(0);
        intakeR = hardwareMap.get(DcMotor.class, "intakeR");
        intakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeR.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setPower(0);

        intakeShifterR = hardwareMap.get(Servo.class, "intakeShifterR");
        intakeShifterR.setDirection(Servo.Direction.FORWARD);
        intakeShifterL = hardwareMap.get(Servo.class, "intakeShifterL");
        intakeShifterL.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
//        colorSensor.setGain(10);

        colorTimer.reset();
    }


    public double getPower(){
        return intakeL.getPower();
    }

    public void simpleIntake(double sumOfTrigs){
        intakeL.setPower(sumOfTrigs);
        intakeR.setPower(sumOfTrigs);
    }

    public void up(){
        intakeShifterL.setPosition(0.05);
        intakeShifterR.setPosition(0.05);
    }
    public void down(){
        intakeShifterL.setPosition(0);
        intakeShifterR.setPosition(0);
    }
    public void shift(){
        intakeShifterL.setPosition(0.4);
        intakeShifterR.setPosition(0.4);
    }


    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float normRed = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue = colors.blue / colors.alpha;

        telemetry.addData("Color R", normRed);
        telemetry.addData("Color G", normGreen);
        telemetry.addData("Color B", normBlue);

        DetectedColor detectedThisFrame = DetectedColor.UNKNOWN;



        boolean objectPresent = normRed < 0.015 && normGreen < 0.028;

        if (objectPresent) {
            if (normGreen > 0.014 && normGreen < 0.025) {
                detectedThisFrame = DetectedColor.GREEN;
            } else {
                detectedThisFrame = DetectedColor.PURPLE;
            }
        }

        if (detectedThisFrame != currentlyDetectedColor) {
            currentlyDetectedColor = detectedThisFrame;
            colorTimer.reset();
        }

        telemetry.addData("Detected Color", currentlyDetectedColor);
        telemetry.addData("Detection Time (s)", colorTimer.seconds());

        if (currentlyDetectedColor != DetectedColor.UNKNOWN && colorTimer.seconds() >= requiredDetectionTimeSeconds) {
            if (currentlyDetectedColor == DetectedColor.PURPLE || currentlyDetectedColor == DetectedColor.GREEN) {
                down();
            }
            return currentlyDetectedColor;
        } else {
            shift();
        }

        return DetectedColor.UNKNOWN;
    }
}