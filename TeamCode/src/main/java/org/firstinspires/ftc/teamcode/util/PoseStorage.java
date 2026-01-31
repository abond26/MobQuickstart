package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Utility class to store and retrieve the robot's final pose from autonomous mode
 * so that teleop can start with the correct position.
 * 
 * Usage:
 * - In autonomous: Call PoseStorage.savePose(follower.getPose()) continuously in loop()
 *                  or at key waypoints to ensure pose is saved even if autonomous ends early
 * - In teleop: Call PoseStorage.loadPose() to get the saved pose, or use default if none saved
 *              Call PoseStorage.calculateRotatorPositionToFaceGoal() to get rotator position
 * 
 * IMPORTANT: Call savePose() continuously in your autonomous loop(), not just at the end!
 *            This ensures the pose is saved even if autonomous ends early due to error/timeout.
 */
//aorjas
public class PoseStorage {
    private static Pose savedPose = null;
    private static boolean poseSaved = false;
    private static Integer savedRotatorPosition = null;
    private static boolean rotatorPositionSaved = false;
    
    // AprilTag positions for scoring goal (field coordinates in inches)
    // These are the positions where the robot starts at the wall facing the AprilTag
    // Red side AprilTag - robot starts at (117.6, 130) facing 36.5 degrees
    private static final double RED_APRILTAG_X = 117.6;
    private static final double RED_APRILTAG_Y = 130.0;
    
    // Blue side AprilTag - robot starts at (24.4, 126.7) facing 143 degrees
    private static final double BLUE_APRILTAG_X = 24.4;
    private static final double BLUE_APRILTAG_Y = 126.7;
    
    // Rotator conversion constants (matches TestingNewAdjustment.java)
    private static final int MOTOR_180_RANGE = 630; // Ticks for 180 degrees of rotation
    
    /**
     * Save the robot's current pose. 
     * 
     * RECOMMENDED: Call this continuously in your autonomous loop() method,
     *              not just at the end, so pose is saved even if autonomous ends early.
     * 
     * @param pose The pose to save
     */
    public static void savePose(Pose pose) {
        if (pose != null) {
            savedPose = pose;
            poseSaved = true;
        }
    }
    
    /**
     * Load the saved pose, or return a default pose if none was saved
     * @param defaultPose The default pose to use if no pose was saved (e.g., new Pose(0, 0, 0))
     * @return The saved pose, or the default pose if none was saved
     */
    public static Pose loadPose(Pose defaultPose) {
        if (poseSaved && savedPose != null) {
            return savedPose;
        }
        return defaultPose;
    }
    
    /**
     * Check if a pose has been saved
     * @return true if a pose was saved, false otherwise
     */
    public static boolean hasSavedPose() {
        return poseSaved && savedPose != null;
    }
    
    /**
     * Clear the saved pose (useful for testing or if you want to reset)
     */
    public static void clearPose() {
        savedPose = null;
        poseSaved = false;
    }
    
    /**
     * Get the saved pose directly (returns null if none saved)
     * @return The saved pose, or null if none was saved
     */
    public static Pose getSavedPose() {
        return savedPose;
    }
    
    /**
     * Calculate the rotator position (in ticks) needed to face the scoring goal
     * based on the saved robot pose.
     * 
     * This uses the same calculation logic as TestingNewAdjustment.calculateAngleToAprilTag()
     * and converts the angle to rotator ticks.
     * 
     * @param isRedSide true if on red side, false if on blue side
     * @param motor180Range the motor ticks for 180 degrees (default 630, some use 910)
     * @param offset the offset in ticks to apply (close12red/blue uses 28, TestingNewAdjustment uses 14)
     * @return rotator position in ticks to face the goal, or 0 if no pose saved
     */
    public static int calculateRotatorPositionToFaceGoal(boolean isRedSide, int motor180Range, int offset) {
        if (!poseSaved || savedPose == null) {
            return 0; // No pose saved, return 0 (center position)
        }
        
        // Get AprilTag position based on side
        double aprilTagX = isRedSide ? RED_APRILTAG_X : BLUE_APRILTAG_X;
        double aprilTagY = isRedSide ? RED_APRILTAG_Y : BLUE_APRILTAG_Y;
        
        // Get robot pose
        double robotX = savedPose.getX();
        double robotY = savedPose.getY();
        double robotHeading = savedPose.getHeading();
        
        // Calculate delta to AprilTag
        double deltaX = aprilTagX - robotX;
        double deltaY = aprilTagY - robotY;
        
        // Calculate absolute angle to AprilTag (in radians)
        // atan2(y, x) gives angle from positive x-axis
        double angleToTag = Math.atan2(deltaY, deltaX);
        
        // Calculate relative angle (how much to rotate from robot's current heading)
        // This is the angle the turret needs to point relative to robot's front
        double relativeAngle = angleToTag - robotHeading;
        
        // Normalize to [-PI, PI]
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;
        
        // Convert to degrees
        double angleDeg = Math.toDegrees(relativeAngle);
        
        // Convert angle to rotator ticks (semicircle: 180 degrees = motor180Range)
        // Same formula as TestingNewAdjustment.adjustRotatorWithLocalization()
        double fracOfSemiCircum = Math.toRadians(angleDeg) / Math.PI;
        int rotatorTicks = (int) (fracOfSemiCircum * motor180Range);
        
        // Apply offset (subtract offset - matches close12red/blue logic)
        rotatorTicks -= offset;
        
        return rotatorTicks;
    }
    
    /**
     * Get debug information about the rotator calculation (for telemetry/debugging).
     * Returns a string with calculation details.
     * 
     * @param isRedSide true if on red side, false if on blue side
     * @param motor180Range the motor ticks for 180 degrees
     * @param offset the offset in ticks
     * @return debug string with calculation details, or "No pose saved" if no pose
     */
    public static String getRotatorCalculationDebug(boolean isRedSide, int motor180Range, int offset) {
        if (!poseSaved || savedPose == null) {
            return "No pose saved";
        }
        
        double aprilTagX = isRedSide ? RED_APRILTAG_X : BLUE_APRILTAG_X;
        double aprilTagY = isRedSide ? RED_APRILTAG_Y : BLUE_APRILTAG_Y;
        double robotX = savedPose.getX();
        double robotY = savedPose.getY();
        double robotHeading = savedPose.getHeading();
        
        double deltaX = aprilTagX - robotX;
        double deltaY = aprilTagY - robotY;
        double angleToTag = Math.atan2(deltaY, deltaX);
        double relativeAngle = angleToTag - robotHeading;
        
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;
        
        double angleDeg = Math.toDegrees(relativeAngle);
        double fracOfSemiCircum = Math.toRadians(angleDeg) / Math.PI;
        int rotatorTicks = (int) (fracOfSemiCircum * motor180Range);
        int finalTicks = rotatorTicks - offset;
        
        return String.format("Robot: (%.1f, %.1f, %.1f°) | Tag: (%.1f, %.1f) | Angle: %.1f° | Ticks: %d (final: %d)", 
            robotX, robotY, Math.toDegrees(robotHeading), aprilTagX, aprilTagY, angleDeg, rotatorTicks, finalTicks);
    }
    
    /**
     * Calculate the rotator position using default motor180Range of 630 and default offset of 28
     * @param isRedSide true if on red side, false if on blue side
     * @return rotator position in ticks to face the goal
     */
    public static int calculateRotatorPositionToFaceGoal(boolean isRedSide) {
        return calculateRotatorPositionToFaceGoal(isRedSide, MOTOR_180_RANGE, 28);
    }
    
    /**
     * Calculate the rotator position using default offset of 28
     * @param isRedSide true if on red side, false if on blue side
     * @param motor180Range the motor ticks for 180 degrees
     * @return rotator position in ticks to face the goal
     */
    public static int calculateRotatorPositionToFaceGoal(boolean isRedSide, int motor180Range) {
        return calculateRotatorPositionToFaceGoal(isRedSide, motor180Range, 28);
    }
    
    /**
     * Save the rotator position (in ticks).
     * Call this in autonomous's "done" state after setting rotator to face goal.
     * 
     * @param rotatorPosition the rotator position in ticks to save
     */
    public static void saveRotatorPosition(int rotatorPosition) {
        savedRotatorPosition = rotatorPosition;
        rotatorPositionSaved = true;
    }
    
    /**
     * Load the saved rotator position, or calculate it from saved pose if not saved.
     * 
     * THIS IS THE KEY METHOD FOR TELEOP - it automatically calculates rotator position
     * to face the goal based on the saved pose, so the rotator will face the goal
     * when teleop starts, even if autonomous didn't set it correctly.
     * 
     * @param isRedSide true if on red side, false if on blue side
     * @param motor180Range the motor ticks for 180 degrees (default 630, some use 910)
     * @param offset the offset in ticks (close12red/blue uses 28, TestingNewAdjustment uses 14)
     * @param defaultPosition the default position to use if nothing saved (usually 0)
     * @return the saved rotator position, calculated position from pose, or default
     */
    public static int loadRotatorPosition(boolean isRedSide, int motor180Range, int offset, int defaultPosition) {
        // If rotator position was explicitly saved, use it
        if (rotatorPositionSaved && savedRotatorPosition != null) {
            return savedRotatorPosition;
        }
        
        // Otherwise, AUTOMATICALLY calculate from saved pose to face the goal
        // This ensures rotator faces goal when teleop starts, even if auton didn't set it
        if (poseSaved && savedPose != null) {
            return calculateRotatorPositionToFaceGoal(isRedSide, motor180Range, offset);
        }
        
        // Fallback to default (only if no pose was saved)
        return defaultPosition;
    }
    
    /**
     * Load the saved rotator position using default offset of 28
     * @param isRedSide true if on red side, false if on blue side
     * @param motor180Range the motor ticks for 180 degrees
     * @param defaultPosition the default position to use if nothing saved (usually 0)
     * @return the saved or calculated rotator position
     */
    public static int loadRotatorPosition(boolean isRedSide, int motor180Range, int defaultPosition) {
        return loadRotatorPosition(isRedSide, motor180Range, 28, defaultPosition);
    }
    
    /**
     * Load the saved rotator position using default motor180Range of 630 and offset of 28
     * @param isRedSide true if on red side, false if on blue side
     * @param defaultPosition the default position to use if nothing saved (usually 0)
     * @return the saved or calculated rotator position
     */
    public static int loadRotatorPosition(boolean isRedSide, int defaultPosition) {
        return loadRotatorPosition(isRedSide, MOTOR_180_RANGE, 28, defaultPosition);
    }
    
    /**
     * Clear the saved rotator position
     */
    public static void clearRotatorPosition() {
        savedRotatorPosition = null;
        rotatorPositionSaved = false;
    }
    
    /**
     * Clear both pose and rotator position
     */
    public static void clearAll() {
        clearPose();
        clearRotatorPosition();
    }
}
