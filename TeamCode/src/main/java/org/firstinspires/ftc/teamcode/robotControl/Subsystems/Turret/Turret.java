package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret implements TurretConstants {
    private DcMotorEx jollyCrusader;
    private DcMotor rotator, theWheelOfTheOx;
    private Servo hood;
    public static int timesCalled = 0;
    public enum ShotType {
        CLOSE,
        MID,
        FAR
    }


    public Turret(@NonNull HardwareMap hardwareMap) {
        //Launcher motor
        jollyCrusader = hardwareMap.get(DcMotorEx.class, "launcher");
        jollyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jollyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jollyCrusader.setDirection(DcMotorSimple.Direction.FORWARD);
        jollyCrusader.setVelocity(0);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, 0, 0, shooterF);
        jollyCrusader.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //X direction motor
        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(0.2);

        //This means counterclockwise is negative
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);

        //Motor that pushes balls into launcher
        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theWheelOfTheOx.setPower(0);
        theWheelOfTheOx.setDirection(DcMotorSimple.Direction.REVERSE);

        //Our precious lil hoody hood
        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(SCALE_RANGE_LOWER, SCALE_RANGE_UPPER);
    }



    //FUNCTIONS FOR ROTATOR
    public void setRotatorPos(int ticks){
        rotator.setTargetPosition(ticks);
    }
    public int getRotatorPos(){
        return rotator.getCurrentPosition();
    }
    public void shiftRotator(int ticks){
        int newPos = getRotatorPos()+ticks;
        setRotatorPos(newPos);
    }
    //Renamed from setRotatorToTurretAngle to make more sense
    public void setRotatorToAngle(double turretAngleDeg) {
        double fracOf180 = Math.toRadians(turretAngleDeg) / Math.PI;
        int targetTicks = ROTATOR_ZERO_TICKS + (int) (fracOf180 * rotator180Range);
        // Optional: clamp to physical limits (e.g. ±270°)
        setRotatorPos(targetTicks);
    }





    //FUNCTIONS FOR LAUNCHER
    public void setVelocity(double velocity){
        jollyCrusader.setVelocity(velocity);
    }
    public double getVelocity(){
        return jollyCrusader.getVelocity();
    }
    public void shiftVelocity(double ticksPerSec){
        double newVelo = getVelocity()+ticksPerSec;
        setVelocity(newVelo);
    }
    public void presetVelo(@NonNull ShotType dist){
        switch (dist){
            case CLOSE:
                setVelocity(CLOSE_VELOCITY);
                break;
            case MID:
                setVelocity(MID_VELOCITY);
                break;
            case FAR:
                setVelocity(FAR_VELOCITY);
                break;
        }
    }

    /*Difference between this function and the
    last one is that this is much easier to iterate through
    whereas the presetVelo is more readable*/
    public void presetVeloSwitch(int dist){
        while (dist > 4) {
            dist-=4;
        }
        switch (dist){
            case 1:
                setVelocity(CLOSE_VELOCITY);
                break;
            case 2:
                setVelocity(MID_VELOCITY);
                break;
            case 3:
                setVelocity(FAR_VELOCITY);
                break;
            case 4:
                setVelocity(0);
                break;
        }
    }




    //FUNCTIONS FOR HOOD
    public void setHoodPos(double pos){
        hood.setPosition(pos);
    }
    public double getHoodPos(){
        return hood.getPosition();
    }
    public void shiftHood(double ticks){
        double newPos = getHoodPos()+ticks;
        setHoodPos(newPos);
    }




    //FUNCTIONS FOR WHEEL OF THE OX
    public void setFeedPower(double power){
        theWheelOfTheOx.setPower(power);
    }
}
