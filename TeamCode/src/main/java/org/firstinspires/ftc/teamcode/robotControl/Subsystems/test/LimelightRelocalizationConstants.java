package org.firstinspires.ftc.teamcode.robotControl.Subsystems.test;

/**
 * Constants for Limelight AprilTag relocalization.
 *
 * COORDINATE CONVERSION:
 * Limelight botpose returns field-center origin in METERS (0,0 = center of field).
 * Pedro Pathing uses INCHES with origin at the bottom-left corner of the field.
 * A standard FTC field is 144 x 144 inches (12 x 12 feet).
 *
 * To convert:  pedroPose = limelightPose_inches + (fieldSize / 2)
 *
 * NOTE: You may need to calibrate FIELD_OFFSET_X and FIELD_OFFSET_Y
 *       if your Pedro Pathing origin is not exactly at the field corner.
 */
public interface LimelightRelocalizationConstants {

    // ═══════════════════════════════════════════════════════════════
    // UNIT CONVERSION
    // ═══════════════════════════════════════════════════════════════
    double METERS_TO_INCHES = 39.3701;

    // ═══════════════════════════════════════════════════════════════
    // COORDINATE TRANSFORM: Limelight → Pedro Pathing
    // ═══════════════════════════════════════════════════════════════
    // Standard FTC field is 144 x 144 inches.
    // Limelight origin = field center → Pedro origin = bottom-left corner.
    // So we add half the field size to shift the origin.
    double FIELD_SIZE_INCHES = 144.0;
    double FIELD_OFFSET_X = FIELD_SIZE_INCHES / 2.0; // 72 inches
    double FIELD_OFFSET_Y = FIELD_SIZE_INCHES / 2.0; // 72 inches

    // ═══════════════════════════════════════════════════════════════
    // QUALITY FILTERS — reject bad AprilTag readings
    // ═══════════════════════════════════════════════════════════════

    /** Max allowed tag ambiguity (0 = perfect, 1 = completely ambiguous).
     *  Reject readings with ambiguity above this. */
    double MAX_POSE_AMBIGUITY = 0.7;

    /** Max allowed distance (inches) between Limelight pose and current
     *  Pedro Pathing pose. Rejects wild jumps. */
    double MAX_POSE_JUMP_INCHES = 144;

    /** Minimum tag area (fraction of image) — reject tags that are too
     *  small / far away to give reliable pose. */
    double MIN_TAG_AREA = 0.001;

    /** Minimum number of fiducials (AprilTags) that must be visible
     *  for a reading to be trusted. 1 = allow single-tag, 2 = require multi-tag. */
    int MIN_FIDUCIALS_REQUIRED = 1;

    // ═══════════════════════════════════════════════════════════════
    // ROTATOR (TURRET) ENCODER CONSTANTS
    // ═══════════════════════════════════════════════════════════════
    // From TurretConstants: 624 ticks = 180 degrees
    int ROTATOR_180_TICKS = 624;
    double TICKS_PER_DEGREE = (double) ROTATOR_180_TICKS / 180.0; // ~3.467
}

