package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class Intake implements IntakeConstants{
//    private NormalizedColorSensor colorSensor;
    private DcMotor intake;
    private Servo intakeShifterR, intakeShifterL;
//
    public Intake(@NonNull HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "tree");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(0);

        intakeShifterR = hardwareMap.get(Servo.class, "intakeShifterR");
        intakeShifterR.setDirection(Servo.Direction.FORWARD);
        intakeShifterL = hardwareMap.get(Servo.class, "intakeShifterL");
        intakeShifterL.setDirection(Servo.Direction.REVERSE);


//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
//        if (colorSensor instanceof SwitchableLight) {
//            ((SwitchableLight)colorSensor).enableLight(true);
//        }

    }


    public double getPower(){
        return intake.getPower();
    }

    public void simpleIntake(double sumOfTrigs){
        intake.setPower(sumOfTrigs);
    }

    public void up(){
        intakeShifterL.setPosition(0.05);
        intakeShifterR.setPosition(0.05);
    }
    public void down(){
        intakeShifterL.setPosition(0);
        intakeShifterR.setPosition(0);
    }
    public void shift(double pos){
        intakeShifterL.setPosition(pos);
        intakeShifterR.setPosition(pos);
    }

}
//