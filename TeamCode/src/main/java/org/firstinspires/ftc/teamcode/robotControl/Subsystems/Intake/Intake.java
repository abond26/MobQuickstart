package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements IntakeConstants{
    private DcMotor intakeL, intakeR;
    private Servo intakeShifterR, intakeShifterL;
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    // Color detection state
    private ElapsedTime colorTimer = new ElapsedTime();
    private ElapsedTime autonColorTimer = new ElapsedTime();
    private DetectedColor currentlyDetectedColor = DetectedColor.UNKNOWN;
    public double requiredDetectionTimeSeconds = 0.3;
    public double autonTime = 0.15;

    public enum DetectedColor {
        PURPLE,
        GREEN,
        BALL,
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
//        colorSensor.setGain(5);

        colorTimer.reset();
    }


    public double getPower(){
        return intakeL.getPower();
    }

    public void simpleIntake(double sumOfTrigs){
        intakeL.setPower(sumOfTrigs);
        intakeR.setPower(sumOfTrigs);
    }
    public void powerON()
    {
        intakeL.setPower(-1);
        intakeR.setPower(-1);
    }
    public void powerOff()
    {
        intakeL.setPower(0);
        intakeR.setPower(0);
    }

    public void up(){
        intakeShifterL.setPosition(0.1);
        intakeShifterR.setPosition(0.1);
    }
    public void down(){
        intakeShifterL.setPosition(0.6);
        intakeShifterR.setPosition(0.6);
    }
    public void shift(){
        intakeShifterL.setPosition(0.5039);
        intakeShifterR.setPosition(0.5039);
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
    public DetectedColor AutonColor(Telemetry telemetry) {
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
            autonColorTimer.reset();
        }

        telemetry.addData("Detected Color", currentlyDetectedColor);
        telemetry.addData("Detection Time (s)", autonColorTimer.seconds());

        if (currentlyDetectedColor != DetectedColor.UNKNOWN && autonColorTimer.seconds() >= autonTime) {
            if (currentlyDetectedColor == DetectedColor.PURPLE || currentlyDetectedColor == DetectedColor.GREEN) {
                down();
            }
            return currentlyDetectedColor;
        } else {
            shift();
        }

        return DetectedColor.UNKNOWN;
    }

    public DetectedColor getDetectedColorByDistance(Telemetry telemetry) {
        double distanceMM = distanceSensor.getDistance(DistanceUnit.MM);
        telemetry.addData("Distance (mm)", distanceMM);

        DetectedColor detectedThisFrame = (distanceMM < 25) ? DetectedColor.BALL : DetectedColor.UNKNOWN;

        if (detectedThisFrame != currentlyDetectedColor) {
            currentlyDetectedColor = detectedThisFrame;
            colorTimer.reset();
        }

        telemetry.addData("Detected State", currentlyDetectedColor);
        telemetry.addData("Detection Time (s)", colorTimer.seconds());

        if (currentlyDetectedColor != DetectedColor.UNKNOWN && colorTimer.seconds() >= requiredDetectionTimeSeconds) {
            down();
            return currentlyDetectedColor;
        } else {
            shift();
        }

        return DetectedColor.UNKNOWN;
    }
}