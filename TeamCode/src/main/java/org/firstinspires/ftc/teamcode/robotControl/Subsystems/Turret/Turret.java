package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret implements TurretConstants {
    private DcMotorEx jollyCrusader, gloomyCrusader;
    private DcMotor rotator;
    private Servo hood;
    public static int timesCalled = 0;
    
    private double targetVelocity = 0;

    public enum ShotType {
        CLOSE,
        MID,
        FAR
    }

    public Turret(@NonNull HardwareMap hardwareMap) {
        // LEFTm
        jollyCrusader = hardwareMap.get(DcMotorEx.class, "launcherL");
        jollyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jollyCrusader.setDirection(DcMotorSimple.Direction.FORWARD);
        jollyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, 0, 0, shooterF);
        jollyCrusader.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        gloomyCrusader = hardwareMap.get(DcMotorEx.class, "launcherR");
        gloomyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gloomyCrusader.setDirection(DcMotorSimple.Direction.REVERSE);
        gloomyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(shooterP, 0, 0, shooterF);
        gloomyCrusader.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);

        // X direction motor
        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(rotatorPower);

        // This means counterclockwise movement on the rotator (not the motor) is
        // negative
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);

        // Our precious lil hoody hood
        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(SCALE_RANGE_LOWER, SCALE_RANGE_UPPER);
    }

    // FUNCTIONS FOR ROTATOR
    /**
     * Set rotator motor power (e.g. 1.0 for full speed). Use in auton for faster
     * aiming.
     */
    public void setRotatorPower(double power) {
        rotator.setPower(power);
    }

    public void resetEncoders() {
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //

    public void setRotatorPos(int ticks) {
        rotator.setTargetPosition(ticks);
    }

    public int getRotatorPos() {
        return rotator.getCurrentPosition();
    }

    public void shiftRotator(int ticks) {
        int newPos = getRotatorPos() + ticks;
        setRotatorPos(newPos);
    }

    // Renamed from setRotatorToTurretAngle to make more sense
    public void setRotatorToAngle(double turretAngleDeg) {
        double fracOf180 = Math.toRadians(turretAngleDeg) / Math.PI;
        int targetTicks = ROTATOR_ZERO_TICKS + (int) (fracOf180 * rotator180Range);
        setRotatorPos(targetTicks);
    }

    // FUNCTIONS FOR LAUNCHER
    public void setVelocity(double velocity) {
        targetVelocity = velocity;
        jollyCrusader.setVelocity(velocity);
        gloomyCrusader.setVelocity(velocity);
    }

    public double[] getVelocity() {
        double velos[] = { jollyCrusader.getVelocity(), gloomyCrusader.getVelocity() };
        return velos;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void shiftVelocity(double ticksPerSec) {
        double newVelo = targetVelocity + ticksPerSec;
        setVelocity(newVelo);
    }

    public void presetVelo(@NonNull ShotType dist) {
        switch (dist) {
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

    /*
     * Difference between this function and the
     * last one is that this is much easier to iterate through
     * whereas the presetVelo is more readable
     */
    public void presetVeloSwitch(int dist) {
        while (dist > 4) {
            dist -= 4;
        }
        switch (dist) {
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

    // FUNCTIONS FOR HOOD
    public void setHoodPos(double pos) {
        hood.setPosition(pos);
    }

    public double getHoodPos() {
        return hood.getPosition();
    }

    public void shiftHood(double ticks) {
        double newPos = getHoodPos() + ticks;
        setHoodPos(newPos);
    }
}