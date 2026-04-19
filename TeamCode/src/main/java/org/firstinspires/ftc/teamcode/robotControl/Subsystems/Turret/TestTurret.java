package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret;

import androidx.annotation.NonNull;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class TestTurret extends Turret {
    // Small coefficients for the manual power loop
    public static PIDFCoefficients FLYWHEEL_PID_CLOSE = new PIDFCoefficients(0.0019, 0, 0.00005, 0.0003);
    public static PIDFCoefficients FLYWHEEL_PID_FAR = new PIDFCoefficients(0.004, 0.00002, 0.00005, 0.0003);
    public static double SWITCH_PID_DIST = 140;

    private DcMotorEx jollyCrusader, gloomyCrusader;
    private Servo rotator, hood;

    private double targetRPM = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    public TestTurret(@NonNull HardwareMap hardwareMap) {
        super(hardwareMap);

        jollyCrusader = hardwareMap.get(DcMotorEx.class, "launcherL");
        jollyCrusader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gloomyCrusader = hardwareMap.get(DcMotorEx.class, "launcherR");
        gloomyCrusader.setDirection(DcMotorSimple.Direction.REVERSE);
        gloomyCrusader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rotator = hardwareMap.get(Servo.class, "rotator");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(SCALE_RANGE_LOWER,SCALE_RANGE_UPPER);
        hood.setPosition(0.8);

        timer.reset();
    }

    public void update(double distance) {
        double currentRPM = getRPM();
        double error = targetRPM - currentRPM;
        double dt = timer.seconds();
        timer.reset();

        if (dt > 0.1) dt = 0.01;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        PIDFCoefficients coeffs = (distance <= SWITCH_PID_DIST) ? FLYWHEEL_PID_CLOSE : FLYWHEEL_PID_FAR;
        double power = (coeffs.f * targetRPM) + (coeffs.p * error) + (coeffs.i * integralSum) + (coeffs.d * derivative);

        power = Math.max(0, Math.min(1, power));
        if (targetRPM == 0) { power = 0; integralSum = 0; }

       jollyCrusader.setPower(power);
            gloomyCrusader.setPower(power);
    }
//

    public double getRPM() {
        return (jollyCrusader.getVelocity() * 60.0) / (ENCODER_CPM * GEAR_RATIO);
    }

    public void setTargetRPM(double rpm) { this.targetRPM = rpm; }
    
    public void setTargetRPMFromDistance(double distance) {
        setTargetRPM(0.121548 * Math.pow(distance, 2) - 8.93555 * distance + 2209.37317);
    }

    public void setVelocity(double ticksPerSec) {
        setTargetRPM((ticksPerSec * 60.0) / (ENCODER_CPM * GEAR_RATIO));
    }



    @Override
    public double getTargetVelocity() { return targetRPM; }

    @Override
    public void shiftVelocity(double rpmStep) { setTargetRPM(targetRPM + rpmStep); }

    @Override
    public void presetVelo(@NonNull ShotType dist) {
        switch (dist) {
            case CLOSE:
                setTargetRPM(CLOSE_VELOCITY);
                break;
            case MID:
                setTargetRPM(MID_VELOCITY);
                break;
            case FAR:
                setTargetRPM(FAR_VELOCITY);
                break;
        }
    }

    @Override
    public void presetVeloSwitch(int dist) {
        while (dist > 4) dist -= 4;
        switch (dist) {
            case 1:
                setTargetRPM(CLOSE_VELOCITY);
                break;
            case 2:
                setTargetRPM(MID_VELOCITY);
                break;
            case 3:
                setTargetRPM(FAR_VELOCITY);
                break;
            case 4:
                setTargetRPM(0);
                break;
        }
    }
}