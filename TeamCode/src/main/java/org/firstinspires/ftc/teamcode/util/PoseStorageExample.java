package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

/**
 * Example showing how to use PoseStorage to pass pose from autonomous to teleop
 * 
 * IN AUTONOMOUS (OPTIONAL - in the "done" state):
 * 
 *     import org.firstinspires.ftc.teamcode.util.PoseStorage;
 *     
 *     // In your loop() method, continuously save pose:
 *     @Override
 *     public void loop() {
 *         follower.update();
 *         statePathUpdate();
 *         
 *         // Continuously save pose (recommended)
 *         PoseStorage.savePose(follower.getPose());
 *     }
 *     
 *     // In your "done" state (OPTIONAL - teleop will calculate if you don't):
 *     case done:
 *         // Save the final pose before autonomous ends
 *         PoseStorage.savePose(follower.getPose());
 *         
 *         // OPTIONAL: Calculate and set rotator to face the goal
 *         // (If you don't do this, teleop will calculate it automatically)
 *         boolean isRedSide = true; // Change to false for blue side
 *         int rotatorPosition = PoseStorage.calculateRotatorPositionToFaceGoal(isRedSide);
 *         rotator.setTargetPosition(rotatorPosition);
 *         PoseStorage.saveRotatorPosition(rotatorPosition);
 *         break;
 * 
 * 
 * IN TELEOP (in runOpMode(), when initializing):
 * 
 *     import org.firstinspires.ftc.teamcode.util.PoseStorage;
 *     import com.qualcomm.robotcore.hardware.DcMotor;
 *     
 *     public void runOpMode() throws InterruptedException {
 *         follower = Constants.createFollower(hardwareMap);
 *         
 *         // Load saved pose from autonomous, or use default if none saved
 *         Pose startPose = PoseStorage.loadPose(new Pose(0, 0, 0));
 *         follower.setStartingPose(startPose);
 *         
 *         // Initialize rotator
 *         rotator = hardwareMap.get(DcMotor.class, "rotator");
 *         rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 *         rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 *         rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
 *         rotator.setPower(1);
 *         
 *         // IMPORTANT: Automatically calculate and set rotator to face the goal
 *         // This will use saved rotator position if available, otherwise calculate from pose
 *         boolean isRedSide = true; // Change to false for blue side
 *         int rotatorPosition = PoseStorage.loadRotatorPosition(isRedSide, 0);
 *         rotator.setTargetPosition(rotatorPosition);
 *         
 *         // DEBUG: See what position was calculated (optional - remove in production)
 *         telemetry.addData("Rotator Position", rotatorPosition);
 *         telemetry.addData("Robot Pose", startPose.getX() + ", " + startPose.getY() + ", " + Math.toDegrees(startPose.getHeading()));
 *         telemetry.addLine(PoseStorage.getRotatorCalculationDebug(isRedSide, 630));
 *         telemetry.update();
 *         
 *         // Or use custom motor180Range if different (e.g., 910 instead of 630):
 *         // int rotatorPosition = PoseStorage.loadRotatorPosition(isRedSide, 910, 0);
 *         
 *         // Now rotator will automatically face the goal when teleop starts!
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
//asfasdf
