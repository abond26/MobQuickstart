package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Turret implements TurretConstants {
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_CLOSE = new PIDFCoefficients(386, 0, 8.7, 13.53);
    public static PIDFCoefficients FLYWHEEL_PIDF_COEFFICIENTS_FAR = new PIDFCoefficients(509, 52, 3.5, 13.53);

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
        // LEFTm
        jollyCrusader = hardwareMap.get(DcMotorEx.class, "launcherL");
        jollyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        jollyCrusader.setDirection(DcMotorSimple.Direction.FORWARD);
        jollyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(386, 0, 8.7, 13.53);
        jollyCrusader.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        gloomyCrusader = hardwareMap.get(DcMotorEx.class, "launcherR");
        gloomyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        gloomyCrusader.setDirection(DcMotorSimple.Direction.REVERSE);
        gloomyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(shooterP, 0, 0,
        // shooterF);
        // gloomyCrusader.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
        // pidfCoefficients2);

        rotator = hardwareMap.get(Servo.class, "rotator");
        rotator.setPosition(0.48965);
        rotator.scaleRange(0.0831, 0.8962);
        rotator.setDirection(Servo.Direction.REVERSE);
        // negativex`
        // Our precious lil hoody hood
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
    //

    public void shiftHood(double ticks) {
        double newPos = getHoodPos() + ticks;
        setHoodPos(newPos);
    }
}