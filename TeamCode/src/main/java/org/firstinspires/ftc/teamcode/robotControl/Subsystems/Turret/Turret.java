package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret implements TurretConstants {
    private DcMotorEx jollyCrusader, gloomyCrusader;
    private Servo rotator;
    private Servo hood;
    public static int timesCalled = 0;
    
    private double targetVelocity = 0;

    public enum ShotType {
        CLOSE,
        MID,
        FAR
    }

    public Turret(@NonNull HardwareMap hardwareMap) {
        // LEFT launcher
        jollyCrusader = hardwareMap.get(DcMotorEx.class, "launcherL");
        jollyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jollyCrusader.setDirection(DcMotorSimple.Direction.FORWARD);
        jollyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // RIGHT launcher
        gloomyCrusader = hardwareMap.get(DcMotorEx.class, "launcherR");
        gloomyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gloomyCrusader.setDirection(DcMotorSimple.Direction.REVERSE);
        gloomyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotator = hardwareMap.get(Servo.class, "rotator");
        rotator.setPosition(0.5);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(SCALE_RANGE_LOWER, SCALE_RANGE_UPPER);
    }

    // FUNCTIONS FOR ROTATOR

    public void setRotatorPos(double pos) {
        rotator.setPosition(pos);
    }

    public double getRotatorPos() {
        return rotator.getPosition();
    }

    public void shiftRotator(double posStep) {
        double newPos = getRotatorPos() + posStep;
        setRotatorPos(newPos);
    }

    public void setRotatorToAngle(double turretAngleDeg) {
        double fracOf180 = Math.toRadians(turretAngleDeg) / Math.PI;
        double targetPos = ROTATOR_ZERO_POS + (fracOf180 * rotator180RangePos);
        setRotatorPos(targetPos);
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
    public double getVelocityL() {
        double velo = jollyCrusader.getVelocity() ;
        return velo;
    }
    public double getVelocityR() {
        double velo = gloomyCrusader.getVelocity() ;
        return velo;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void shiftVelocity(double ticksPerSec) {
        double newVelo = targetVelocity + ticksPerSec;
        setVelocity(newVelo);
    }

    // --- RPM control (feedforward + feedback from Flywheel) ---

    /**
     * Returns the current RPM of the left launcher motor.
     * Uses the same formula as Flywheel.getRPM().
     */
    public double getRPM() {
        double ticksPerSec = jollyCrusader.getVelocity();
        return ((ticksPerSec / ENCODER_CPM) * 60) / GEAR_RATIO;
    }

    /**
     * Commands both launcher motors to a target RPM using a
     * feedforward + proportional feedback loop (ported from Flywheel).
     * Call this every loop iteration for closed-loop RPM control.
     */
    public void setRPM(double targetRPM) {
        double error       = targetRPM - getRPM();
        double feedforward = KV * targetRPM + KS;
        double feedback    = KP * error;
        double power       = feedforward + feedback;
        jollyCrusader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gloomyCrusader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jollyCrusader.setPower(power);
        gloomyCrusader.setPower(power);
    }

    /** Stops both launcher motors. */
    public void stop() {
        jollyCrusader.setPower(0);
        gloomyCrusader.setPower(0);
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