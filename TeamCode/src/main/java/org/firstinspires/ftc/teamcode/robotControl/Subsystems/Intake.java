package org.firstinspires.ftc.teamcode.robotControl.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotor intake;

    public Intake(@NonNull HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotor.class, "tree");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
    }

    public void simpleIntake(double sumOfTrigs){
        intake.setPower(sumOfTrigs);
    }
}
