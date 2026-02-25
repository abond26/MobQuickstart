package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements IntakeConstants{
    private DcMotor intake;

    public Intake(@NonNull HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "tree");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
    }
    public double getPower(){
        return intake.getPower();
    }

    public void simpleIntake(double sumOfTrigs){
        intake.setPower(sumOfTrigs);
    }

}
