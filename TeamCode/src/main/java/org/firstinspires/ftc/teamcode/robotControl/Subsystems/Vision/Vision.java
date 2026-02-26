package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Vision implements VisionConstants{

    private Limelight3A limelight;

    public Vision(@NonNull HardwareMap hardwareMap, int pipeline) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
            limelight.start();
        }
    }

    public boolean hasTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public double getTx() {
        return limelight.getLatestResult().getTx();
    }

    public double getTy() {
        return limelight.getLatestResult().getTy();
    }

    public double getDistance() {
        double tyDeg = getTy();
        double tyRad = Math.abs(Math.toRadians(tyDeg + limelightUpAngle));
        double dist = y / Math.tan(tyRad);
        return dist;
    }
}
