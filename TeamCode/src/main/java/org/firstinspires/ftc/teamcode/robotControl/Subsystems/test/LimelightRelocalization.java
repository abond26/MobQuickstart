package org.firstinspires.ftc.teamcode.robotControl.Subsystems.test;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

/**
 * Limelight AprilTag Relocalization using MegaTag2.
 *
 * PURPOSE:
 * Correct deadwheel localization drift by using AprilTag field-position
 * to determine where the robot ACTUALLY is, then update Pedro Pathing.
 *
 * HOW IT WORKS WITH THE TURRET:
 * The Limelight is mounted on the turret (rotator), which can spin
 * independently of the chassis. MegaTag2 solves this elegantly:
 *   1. We feed the CHASSIS heading (from Pedro Pathing / Pinpoint IMU)
 *      to the Limelight via updateRobotOrientation().
 *   2. MegaTag2 uses this heading + the AprilTag detection to compute
 *      the robot's TRUE field position, regardless of turret angle.
 *   3. We convert the result from Limelight coordinates (meters, field-center)
 *      to Pedro Pathing coordinates (inches, corner origin).
 *
 * USAGE:
 *   // In init:
 *   relocalization = new LimelightRelocalization(hardwareMap, 0);
 *
 *   // In loop:
 *   relocalization.updateOrientation(chassisHeadingDeg);
 *   Pose newPose = relocalization.getRelocalizationPose(currentPedroPose);
 *   if (newPose != null) {
 *       chassisLocal.setPose(newPose);
 *   }
 */
public class LimelightRelocalization implements LimelightRelocalizationConstants {

    private Limelight3A limelight;

    // Tracking for telemetry
    private boolean lastResultValid = false;
    private String lastRejectReason = "No data yet";
    private int relocalizationCount = 0;
    private Pose lastLimelightPose = null;

    // Raw Limelight values for debugging coordinate mapping
    private double rawLimelightX = 0; // inches, field-center origin
    private double rawLimelightY = 0;
    private double rawLimelightZ = 0;
    private double rawHeadingDeg = 0;
    private int lastFiducialCount = 0;
    private double mt1X = 0, mt1Y = 0, mt1Yaw = 0; // MegaTag1 for comparison

    // ─────────────────────────────────────────────────────────────
    // CONSTRUCTOR
    // ─────────────────────────────────────────────────────────────

    /**
     * Initialize the Limelight relocalization using an existing Limelight instance.
     * This avoids double-initialization issues (stuck in stop).
     *
     * @param existingLimelight the Limelight3A from Robot's Vision subsystem
     *                          (get it via robot.vision.getLimelight())
     */
    public LimelightRelocalization(Limelight3A existingLimelight) {
        limelight = existingLimelight;
    }

    // ─────────────────────────────────────────────────────────────
    // CORE METHODS
    // ─────────────────────────────────────────────────────────────

    /**
     * Optional: Feed the camera's true field heading to the Limelight.
     * With MT1, this is NOT strictly required for X/Y pose, but can help
     * the Limelight's internal tracking algorithms.
     */
    public void updateOrientation(double chassisHeadingDeg, double turretRelativeDeg) {
        if (limelight != null) {
            double cameraFieldYaw = chassisHeadingDeg + turretRelativeDeg;
            while (cameraFieldYaw > 180) cameraFieldYaw -= 360;
            while (cameraFieldYaw < -180) cameraFieldYaw += 360;
            limelight.updateRobotOrientation(cameraFieldYaw);
        }
    }

    /**
     * Attempt to relocalize the robot using AprilTags.
     *
     * Returns a corrected Pose in Pedro Pathing coordinates if a valid
     * AprilTag reading is available, or null if no valid reading.
     *
     * @param currentPedroPose the current Pedro Pathing pose (for sanity checking)
     * @param result the cached LLResult polled once per loop in TeleOp
     * @return corrected Pose in Pedro Pathing coords, or null if no valid data
     */
    @Nullable
    public Pose getRelocalizationPose(@NonNull Pose currentPedroPose, @Nullable LLResult result) {
        if (limelight == null) {
            lastResultValid = false;
            lastRejectReason = "Limelight not initialized";
            return null;
        }

        // ── Check 1: Result exists and is valid ──
        if (result == null || !result.isValid()) {
            lastResultValid = false;
            lastRejectReason = "No valid result";
            return null;
        }

        // ── Check 2: Fiducials detected ──
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.size() < MIN_FIDUCIALS_REQUIRED) {
            lastResultValid = false;
            lastRejectReason = "Not enough fiducials: " +
                    (fiducials == null ? 0 : fiducials.size());
            return null;
        }

        // ── Get MT1 (botpose - pure visual) ──
        Pose3D botposeMT1 = result.getBotpose();
        if (botposeMT1 == null) {
            lastResultValid = false;
            lastRejectReason = "No botpose available";
            return null;
        }

        Position p1 = botposeMT1.getPosition().toUnit(DistanceUnit.INCH);
        mt1X = p1.x;
        mt1Y = p1.y;
        mt1Yaw = botposeMT1.getOrientation().getYaw(AngleUnit.DEGREES);

        // We will use MT1 for our position because MT2 injection on a turret is too unstable.
        double limelightX = mt1X; // inches, field-center origin
        double limelightY = mt1Y; // inches, field-center origin
        double limelightZ = p1.z; // inches, height
        double headingRad = Math.toRadians(mt1Yaw);

        // Save raw values for telemetry debugging (we're now using MT1 as raw)
        rawLimelightX = limelightX;
        rawLimelightY = limelightY;
        rawLimelightZ = limelightZ;
        rawHeadingDeg = mt1Yaw;
        lastFiducialCount = fiducials.size();

        // ── Convert from Limelight coords → Pedro Pathing coords ──
        // Limelight (MT1 visual): origin = field center
        //   +X = Right (Red Alliance wall to Blue Alliance wall)
        //   +Y = Forward (Audience to Backdrop)
        // Pedro Pathing: origin = bottom-left corner
        //   +X = Right
        //   +Y = Forward

        // Since Pedro is corner-origin, we add 72 (FIELD_OFFSET).
        // The MT1 data (X=16.5, Y=25.0) correctly maps to Pedro (88.5, 97.0)
        double pedroX = limelightX + FIELD_OFFSET_X;
        double pedroY = limelightY + FIELD_OFFSET_Y;

        // Note: For heading, we ALWAYS keep the Pedro Pathing (deadwheel/IMU) heading.
        // The Limelight's MT1 Yaw represents the *camera's* yaw (turret inclusive),
        // not the chassis heading. So we use currentPedroPose.getHeading() below.
        double chassisHeadingRad = currentPedroPose.getHeading();

        Pose correctedPose = new Pose(pedroX, pedroY, chassisHeadingRad);

        // Always save the computed pose so we can see it in telemetry, even if it's rejected
        lastLimelightPose = correctedPose;

        // ── Check 4: Sanity check — reject if pose jumps too far ──
        double dx = pedroX - currentPedroPose.getX();
        double dy = pedroY - currentPedroPose.getY();
        double jumpDistance = Math.sqrt(dx * dx + dy * dy);

        if (jumpDistance > MAX_POSE_JUMP_INCHES) {
            lastResultValid = false;
            lastRejectReason = String.format("Jump too large: %.1f in", jumpDistance);
            return null;
        }

        // ── All checks passed — build corrected pose ──

        lastResultValid = true;
        lastRejectReason = "OK";
        relocalizationCount++;

        return correctedPose;
    }

    // ─────────────────────────────────────────────────────────────
    // UTILITY METHODS
    // ─────────────────────────────────────────────────────────────

    /**
     * Convert rotator encoder ticks to degrees.
     * Useful for telemetry to verify turret angle.
     */
    public static double rotatorTicksToDegrees(int ticks) {
        return ticks / TICKS_PER_DEGREE;
    }

    /**
     * Check if the Limelight is connected and running.
     */
    public boolean isConnected() {
        return limelight != null;
    }

    /**
     * Get the number of successful relocalizations since start.
     */
    public int getRelocalizationCount() {
        return relocalizationCount;
    }

    /**
     * Get the last Limelight-derived pose (even if it was rejected).
     * Returns null if no pose has ever been computed.
     */
    @Nullable
    public Pose getLastLimelightPose() {
        return lastLimelightPose;
    }

    // ─────────────────────────────────────────────────────────────
    // TELEMETRY
    // ─────────────────────────────────────────────────────────────

    /**
     * Add relocalization debug data to telemetry.
     * Call this in your loop for real-time debugging.
     */
    public void addTelemetry(@NonNull Telemetry telemetry, @NonNull Pose currentPedroPose) {
        telemetry.addLine("═══ RELOCALIZATION ═══");
        telemetry.addData("LL Connected", isConnected());
        telemetry.addData("Last Result Valid", lastResultValid);
        telemetry.addData("Reject Reason", lastRejectReason);
        telemetry.addData("Total Relocalizations", relocalizationCount);

        telemetry.addLine("─── Pedro Pose ───");
        telemetry.addData("Pedro X", "%.1f", currentPedroPose.getX());
        telemetry.addData("Pedro Y", "%.1f", currentPedroPose.getY());
        telemetry.addData("Pedro Heading", "%.1f°",
                Math.toDegrees(currentPedroPose.getHeading()));

        if (lastLimelightPose != null) {
            telemetry.addLine("─── Limelight Pose (converted) ───");
            telemetry.addData("LL X", "%.1f", lastLimelightPose.getX());
            telemetry.addData("LL Y", "%.1f", lastLimelightPose.getY());
            telemetry.addData("LL Heading", "%.1f°",
                    Math.toDegrees(lastLimelightPose.getHeading()));

            double dx = lastLimelightPose.getX() - currentPedroPose.getX();
            double dy = lastLimelightPose.getY() - currentPedroPose.getY();
            telemetry.addData("Pose Difference", "%.1f in",
                    Math.sqrt(dx * dx + dy * dy));
        }

        // RAW VALUES — use these to figure out coordinate mapping
        telemetry.addLine("─── RAW MT1 Limelight (pure visual) ───");
        telemetry.addData("MT1 LL X", "%.1f", mt1X);
        telemetry.addData("MT1 LL Y", "%.1f", mt1Y);
        telemetry.addData("MT1 LL Yaw", "%.1f°", mt1Yaw);
        telemetry.addData("Fiducials seen", lastFiducialCount);
    }

    /**
     * Stop the Limelight (call in stop()).
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}