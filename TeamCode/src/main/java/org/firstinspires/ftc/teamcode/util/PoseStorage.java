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
 * 
 * IMPORTANT: Call savePose() continuously in your autonomous loop(), not just at the end!
 *            This ensures the pose is saved even if autonomous ends early due to error/timeout.
 */
public class PoseStorage {
    private static Pose savedPose = null;
    private static boolean poseSaved = false;
    
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
}
