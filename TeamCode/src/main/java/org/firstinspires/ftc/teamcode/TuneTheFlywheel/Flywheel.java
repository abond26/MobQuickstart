package org.firstinspires.ftc.teamcode.TuneTheFlywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private DcMotorEx m1,m2;

    private double encoderCPM = 28;

    private double kV = 0.000196 ,kS=0.145, kP=0.0015000;


    public void init(HardwareMap hardwareMap){
        m1 = hardwareMap.get(DcMotorEx.class,"left motor");
        m2 = hardwareMap.get(DcMotorEx.class,"right motor");
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);


    }
    public void setMotorPower(double power) {
        m1.setPower(power);
        m2.setPower(power);
    }
    public void stopMotors(){
        m1.setPower(0);
        m2.setPower(0);
    }
    public void setMotorRPM(double targetRPM) {
        double error = targetRPM - getRPM();
        double feedforward = kV * targetRPM + kS;
        double feedback = kP * error;
        double power = feedforward + feedback;
        m1.setPower(power);
        m2.setPower(power);
    }

    public double getTicksPerSec() {
        return m1.getVelocity();
    }

    public double getRPM() {
        return ((getTicksPerSec() / encoderCPM) * 60) / 1.57 ;
    }


}
