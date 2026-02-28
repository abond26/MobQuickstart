package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

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



    @Nullable
    public Pose getPoseLimelight() {
        if (limelight == null) {
            return null;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            return null;
        }
        Position pos = botpose.getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles orient = botpose.getOrientation();
        double x = pos.x;
        double y = pos.y;
        double heading = orient.getYaw(AngleUnit.RADIANS);
        return new Pose(x, y, heading);
    }

    private static double getFiducialAmbiguity(LLResultTypes.FiducialResult fr) {
        try {
            java.lang.reflect.Method m = fr.getClass().getMethod("getAmbiguity");
            Object v = m.invoke(fr);
            return v instanceof Number ? ((Number) v).doubleValue() : -1;
        } catch (Exception e) {
            return -1;
        }
    }

    private static double getFiducialDistToCamera(LLResultTypes.FiducialResult fr) {
        try {
            java.lang.reflect.Method m = fr.getClass().getMethod("getDistToCamera");
            Object v = m.invoke(fr);
            return v instanceof Number ? ((Number) v).doubleValue() : 0;
        } catch (Exception e) {
            return 0;
        }
    }
}
