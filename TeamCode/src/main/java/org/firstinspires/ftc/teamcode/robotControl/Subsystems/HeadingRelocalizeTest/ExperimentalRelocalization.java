package org.firstinspires.ftc.teamcode.robotControl.Subsystems.HeadingRelocalizeTest;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.test.LimelightRelocalization;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.test.LimelightRelocalizationConstants;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TurretConstants;

/**
 * Experimental version of LimelightRelocalization.
 * Includes the TURRET SYNC math for fixing gear slippage.
 */
public class ExperimentalRelocalization extends LimelightRelocalization implements LimelightRelocalizationConstants, TurretConstants {

    public ExperimentalRelocalization(Limelight3A existingLimelight) {
        super(existingLimelight);
    }

    /**
     * MASTER SYNC: Estimates what the turret encoder ticks SHOULD be.
     * Use this if you suspect the gears have slipped.
     */
    @Nullable
    public Integer getVisualTurretTicks(double chassisHeadingDeg, LLResult result) {
        if (result == null || !result.isValid()) return null;
        Pose3D botpose = result.getBotpose(); // MT1 field pose
        if (botpose == null) return null;

        double cameraFieldYaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

        // Visual Heading = Chassis Heading + Turret Angle
        // Therefore: Turret Angle = Visual Heading - Chassis Heading
        double trueTurretAngleDeg = cameraFieldYaw - chassisHeadingDeg;

        // Normalize to [-180, 180]
        while (trueTurretAngleDeg > 180) trueTurretAngleDeg -= 360;
        while (trueTurretAngleDeg < -180) trueTurretAngleDeg += 360;

        // Convert Angle -> Ticks
        return (int)(trueTurretAngleDeg * TICKS_PER_DEGREE) + ROTATOR_ZERO_TICKS;
    }
}
