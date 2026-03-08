package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Example teleop showing how to use PoseStorage to load pose and rotator position
 * from autonomous. This is a template - adapt it to your actual teleop file.
 */
@TeleOp(name = "Example Teleop with PoseStorage")
public class PoseStorageTeleopExample extends LinearOpMode {
    private Follower follower;
    private DcMotor rotator;
    private int motor180Range = 910; // Match your autonomous (close12red/blue uses 910)
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize follower
        follower = Constants.createFollower(hardwareMap);
        
        // Load saved pose from autonomous, or use default if none saved
        Pose startPose = PoseStorage.loadPose(new Pose(0, 0, 0));
        follower.setStartingPose(startPose);
        
        // Initialize rotator
        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);
        
        // Automatically calculate and set rotator to face the goal
        // true = red side, false = blue side
        boolean isRedSide = true; // Change to false for blue side teleop
        int offset = 28; // Match your autonomous's offset (close12red/blue uses 28)
        int rotatorPosition = PoseStorage.loadRotatorPosition(isRedSide, motor180Range, offset, 0);
        rotator.setTargetPosition(rotatorPosition);
        
        // Optional: Debug telemetry
        if (PoseStorage.hasSavedPose()) {
            telemetry.addData("Starting from", "Autonomous end position");
            telemetry.addData("Rotator Position", rotatorPosition);
            telemetry.addLine(PoseStorage.getRotatorCalculationDebug(isRedSide, motor180Range, offset));
        } else {
            telemetry.addData("Starting from", "Default position (0, 0, 0)");
        }
        telemetry.update();
        
        waitForStart();
        follower.startTeleopDrive();
        
        while (opModeIsActive()) {
            follower.update();
            // Your teleop code here...
        }
    }
}
