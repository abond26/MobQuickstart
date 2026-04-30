package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Configurable

public class Intake implements IntakeConstants{
    private DcMotor intakeL, intakeR;
    private Servo intakeShifterR, intakeShifterL;
    private DistanceSensor distanceSensor;
    
    private ElapsedTime colorTimer = new ElapsedTime();
    private DetectedColor currentlyDetectedColor = DetectedColor.UNKNOWN;
    public static double DISTANCE_THRESHOLD_INCHES = 1.8;


    public double requiredDetectionTimeSeconds = 0.3;
    public double autonTime = 0.3;

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
        intakeShifterR.setPosition(0.582);
        intakeShifterL = hardwareMap.get(Servo.class, "intakeShifterL");
        intakeShifterL.setDirection(Servo.Direction.REVERSE);
        intakeShifterL.setPosition(0.582);


        distanceSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
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
        intakeShifterL.setPosition(0.7);
        intakeShifterR.setPosition(0.7);
    }
    public void shift(){
        intakeShifterL.setPosition(0.582);
        intakeShifterR.setPosition(0.582);
    }
    public void gateCollet()
    {
        intakeShifterL.setPosition(0.57);
        intakeShifterR.setPosition(0.57);
    }


    public DetectedColor getDetectedBall(Telemetry telemetry) {
        double distanceInches = distanceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Intake Distance (in)", distanceInches);

        DetectedColor detectedThisFrame = (distanceInches < DISTANCE_THRESHOLD_INCHES) ? DetectedColor.BALL : DetectedColor.UNKNOWN;

        if (detectedThisFrame != currentlyDetectedColor) {
            currentlyDetectedColor = detectedThisFrame;
            colorTimer.reset();
        }

        if (currentlyDetectedColor == DetectedColor.BALL && colorTimer.seconds() >= requiredDetectionTimeSeconds) {
            down();
            return DetectedColor.BALL;
        }
        shift();

        return DetectedColor.UNKNOWN;
    }
//
    public DetectedColor getDetectedColor(Telemetry telemetry) {
        return getDetectedBall(telemetry);
    }

    public DetectedColor AutonColor(Telemetry telemetry) {
         if (distanceSensor.getDistance(DistanceUnit.INCH) < DISTANCE_THRESHOLD_INCHES) {
             return DetectedColor.BALL;
         }
         return DetectedColor.UNKNOWN;
    }

    public DetectedColor getDetectedColorByDistance(Telemetry telemetry) {
        return getDetectedBall(telemetry);
    }
}