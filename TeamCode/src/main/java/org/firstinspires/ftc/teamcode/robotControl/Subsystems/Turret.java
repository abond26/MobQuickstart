package org.firstinspires.ftc.teamcode.robotControl.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Turret implements Constants{
    private DcMotorEx jollyCrusader;

    public Turret(HardwareMap hardwareMap) {
        jollyCrusader = hardwareMap.get(DcMotorEx.class, "launcher");
        jollyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jollyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jollyCrusader.setDirection(DcMotorSimple.Direction.FORWARD);
        jollyCrusader.setVelocity(0);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, 0, 0, shooterF);
        jollyCrusader.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void setVelocity(double distance){
    }

}
