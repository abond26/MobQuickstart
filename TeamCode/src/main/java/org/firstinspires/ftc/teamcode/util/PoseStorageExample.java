package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Example showing how to use PoseStorage to pass pose from autonomous to teleop
 * 
 * IN AUTONOMOUS (at the end, in the "done" state or before stopping):
 * 
 *     import org.firstinspires.ftc.teamcode.util.PoseStorage;
 *     
 *     // In your loop() method, when autonomous is finishing:
 *     case done:
 *         // Save the final pose before autonomous ends
 *         PoseStorage.savePose(follower.getPose());
 *         break;
 * 
 * 
 * IN TELEOP (in runOpMode(), when initializing):
 * 
 *     import org.firstinspires.ftc.teamcode.util.PoseStorage;
 *     
 *     public void runOpMode() throws InterruptedException {
 *         follower = Constants.createFollower(hardwareMap);
 *         
 *         // Load saved pose from autonomous, or use default if none saved
 *         Pose startPose = PoseStorage.loadPose(new Pose(0, 0, 0));
 *         follower.setStartingPose(startPose);
 *         
 *         // Rest of your initialization...
 *     }
 * 
 * 
 * OPTIONAL: Check if pose was saved
 * 
 *     if (PoseStorage.hasSavedPose()) {
 *         telemetry.addData("Starting from", "Autonomous end position");
 *     } else {
 *         telemetry.addData("Starting from", "Default position (0, 0, 0)");
 *     }
 */

public class PoseStorageExample {
    // This is just documentation - see comments above for usage
}
