package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision;

import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Vision implements VisionConstants{

    private Limelight3A limelight;
    public Limelight3A getLimelight() {
        Log.w("get limelight: ", limelight.toString());
        return limelight;
    }
    public Vision(@NonNull HardwareMap hardwareMap, int pipeline) {

        // Try common config names (Driver Station lets you rename the device)
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Log.w("Vision", "Limelight Loaded");

        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
            Log.w("Vision", "Pipeline switched");
            limelight.start();
            Log.w("Vision", "Pipeline inited");
        }
    }

    /** True if the Limelight was found in the robot configuration. */
    public boolean isConnected() {
        return limelight != null;
    }

    public boolean hasTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public double getTx() {
        if (limelight == null) return 0;
        LLResult r = limelight.getLatestResult();
        return (r != null) ? r.getTx() : 0;
    }



    public double getTy() {
        if (limelight == null) return 0;
        LLResult r = limelight.getLatestResult();
        return (r != null) ? r.getTy() : 0;
    }

    public double getDistance() {
        if (limelight == null) return 0;
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

    public void updateOrientation(double headingDegrees) {
        if (limelight != null) {
            limelight.updateRobotOrientation(headingDegrees);
        }
    }

    @Nullable
    public Pose getMegaTag2Pose() {
        if (limelight == null) return null;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D botposeMT2 = result.getBotpose_MT2();
        if (botposeMT2 == null) return null;

        Position pos = botposeMT2.getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles orient = botposeMT2.getOrientation();
        
        double pedroX = pos.y + 72.0;
        double pedroY = -pos.x + 72.0; 
        double heading = orient.getYaw(AngleUnit.RADIANS);
        
        return new Pose(pedroX, pedroY, heading); 
    }

    public Pose3D getRobotPosition(){
        List<FiducialResult> fiducialResult = limelight.getLatestResult().getFiducialResults();
        if (!fiducialResult.isEmpty() && fiducialResult.get(0).getTargetArea() >= 0.01) {
            return fiducialResult.get(0).getRobotPoseFieldSpace();
        }
        return null;
        //
    }
}