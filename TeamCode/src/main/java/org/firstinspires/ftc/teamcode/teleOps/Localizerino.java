package org.firstinspires.ftc.teamcode.teleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class Localizerino extends LinearOpMode {
    private Limelight3A limelight;
    int motor180Range = 630;
    int limelightUpAngle = 20;
    private int limeHeight = 35;
    private int tagHeight = 75;
    private int y = tagHeight - limeHeight;
    private Follower follower;
    private DcMotor rotator;
    private final Pose startPose = new Pose(72, 8, 0);
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private static final double APRILTAG_X = 15.0; // AprilTag X position on field (inches) - UPDATE THIS
    private static final double APRILTAG_Y = 128.0; // AprilTag Y position on field (inches) - UPDATE THIS

    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setPower(1);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        follower.startTeleopDrive();






        while (opModeIsActive()){
            follower.update();
            calculateAngleToAprilTag();
            drive();


            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();
        }
    }






    public double calculateAngleToAprilTag() {
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();

        double deltaX = APRILTAG_X - robotX;
        double deltaY = APRILTAG_Y - robotY;

        // Calculate absolute angle to AprilTag (in radians)
        // atan2(y, x) gives angle from positive x-axis
        double angleToTag = Math.atan2(deltaX, deltaY);

        // Calculate relative angle (how much to rotate from robot's current heading)
        // This is the angle the turret needs to point relative to robot's front
        double relativeAngle = angleToTag + robotHeading;

        // Normalize to [-PI, PI]
//        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
//        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

        // Convert to degrees
        double angleDeg = Math.toDegrees(relativeAngle);

        // Debug telemetry
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Robot Heading Deg", Math.toDegrees(robotHeading));
        telemetry.addData("Delta X", deltaX);
        telemetry.addData("Delta Y", deltaY);
        telemetry.addData("Angle to Tag Rad", angleToTag);
        telemetry.addData("Angle to Tag Deg", Math.toDegrees(angleToTag));
        telemetry.addData("Relative Angle Deg", angleDeg);

        return angleDeg;
    }

    public void drive(){
        double y = -gamepad1.left_stick_y; //forward/backward
        double x = gamepad1.left_stick_x; //strafe (left/right)
        double r = gamepad1.right_stick_x; //rotate

        //bot movements
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        double leftFrontPower = (y + x + r) / denominator;
        double leftRearPower = (y - x + r) / denominator;
        double rightFrontPower = (y - x - r) / denominator;
        double rightRearPower = (y + x - r) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }
}
