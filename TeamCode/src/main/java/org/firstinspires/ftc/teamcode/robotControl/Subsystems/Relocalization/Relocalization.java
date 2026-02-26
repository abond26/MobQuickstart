package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Relocalization;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Chassis.ChassisLocal;

/**
 * Relocalizes chassis heading using the Limelight (on the turret) and turret encoder.
 * All logic is self-contained here; uses HardwareMap to access limelight and rotator.
 *
 * <p>Field convention (FTC): right = 0°, straight = 90°, left = 180°. When tx = 0 the camera
 * is centered on the tag, and the rotator/camera is at {@link #FIELD_ANGLE_WHEN_TX_ZERO}° on the field
 * (e.g. 145° when facing the backboard on blue).
 *
 * <p>Formula: Chassis Heading = Field Camera Angle − Turret Encoder Angle.
 */
public class Relocalization {

    private final ChassisLocal chassis;
    private final Limelight3A limelight;
    private final DcMotor rotator;

    /** When tx = 0 (camera centered on tag), the camera/rotator is at this angle on the field (e.g. blue backboard). */
    public static final double FIELD_ANGLE_WHEN_TX_ZERO = 145.0;

    /** Rotator encoder: ticks at chassis front; ticks per 180° (match Turret hardware). */
    private static final int ROTATOR_ZERO_TICKS = 0;
    private static final int ROTATOR_180_RANGE = 624;

    /** Max allowed heading correction in degrees; larger corrections are ignored. */
    private static final double MAX_HEADING_CORRECTION_DEG = 45.0;

    private static final String LIMELIGHT_NAME = "limelight";
    private static final String ROTATOR_NAME = "rotator";

    public Relocalization(@NonNull HardwareMap hardwareMap, @NonNull ChassisLocal chassis) {
        this.chassis = chassis;
        this.limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        this.rotator = hardwareMap.get(DcMotor.class, ROTATOR_NAME);
    }

    /**
     * If the camera sees an AprilTag, compute chassis heading from camera field angle and
     * turret encoder angle, then update the chassis pose (heading only; x,y unchanged).
     * Call every teleop/auton loop when you want heading relocalization.
     */
    public void tryRelocalizeHeading() {
        if (limelight == null || rotator == null) return;

        Double fieldCameraDeg = getFieldCameraHeadingDeg();
        if (fieldCameraDeg == null) return;

        double turretDeg = getTurretAngleDeg();
        double chassisHeadingDeg = normalizeDegrees(fieldCameraDeg - turretDeg);
        double chassisHeadingRad = Math.toRadians(chassisHeadingDeg);

        Pose current = chassis.getPose();
        double currentHeadingDeg = Math.toDegrees(current.getHeading());
        double correctionDeg = Math.abs(normalizeDegrees(chassisHeadingDeg - currentHeadingDeg));
        if (correctionDeg > MAX_HEADING_CORRECTION_DEG) return;

        Pose corrected = new Pose(current.getX(), current.getY(), chassisHeadingRad);
        chassis.setPose(corrected);
    }

    /** Camera's heading (yaw) on the field in degrees from AprilTag botpose, or null if not valid. */
    private Double getFieldCameraHeadingDeg() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        try {
            Pose3D botpose = result.getBotpose();
            if (botpose == null) return null;
            return botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        } catch (Exception e) {
            return null;
        }
    }

    /** Turret angle in degrees relative to chassis front (positive = turret to the right). */
    private double getTurretAngleDeg() {
        int ticks = rotator.getCurrentPosition() - ROTATOR_ZERO_TICKS;
        return (ticks / (double) ROTATOR_180_RANGE) * 180.0;
    }

    private static double normalizeDegrees(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
