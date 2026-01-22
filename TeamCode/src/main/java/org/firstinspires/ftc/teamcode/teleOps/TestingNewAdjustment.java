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
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp

public class TestingNewAdjustment extends LinearOpMode {
    double newTime;
    double time;


    //282
    //-301
    double sumOfTrigs;
    boolean yLast = false;
    boolean aLast =false;
    boolean xLast = false;
    boolean rightBumperLast = false;
    boolean dpadUpLast = false;
    boolean dpadDownLast = false;
    ElapsedTime rightBumperTimer = new ElapsedTime();
    boolean rightBumperTimerStarted = false;
    boolean localizationAdjustmentActive = false;
    int motor180Range = 630;
    private static final int DEGREES_180_TICKS = 630; // 180 degrees = motor180Range
    private static final int HALFWAY_POSITIVE = 250; // 225 degrees (positive side)
    private static final int HALFWAY_NEGATIVE = -157; // 225 degrees (negative side)

    // Hard limits for rotation motor - if exceeded, reverse direction
    private static final int HARD_LIMIT_POSITIVE = 250; // Maximum positive position
    private static final int HARD_LIMIT_NEGATIVE = -157; // Maximum negative position

    private double targetVelocity = 0;
    private boolean hasRumbledForVelocity = false;
    private static final double VELOCITY_LENIANCE = 20.0;
    int limelightUpAngle = 20;
    private int limeHeight = 35;
    private int tagHeight = 75;
    private int y = tagHeight - limeHeight;
    public static double driveMultiplier = 1;
    private Limelight3A limelight;
    public ElapsedTime runtime = new ElapsedTime();
    private Servo hood;

    // Distance threshold for hood adjustment (tune this value)
    private static final double FIRST_DISTANCE_THRESHOLD = 140.0;
    private static final double SECOND_DISTANCE_THRESHOLD = 200;
    private static final double CLOSE_HOOD_POSITION = 0.0339; // Hood position for close shots
    private static final double MID_HOOD_POSITION = 0.203+0.0167;
    private static final double FAR_HOOD_POSITION = 0.25+.0129; // Hood position for far shots

    // AprilTag position in reset coordinate system (bot starts at 0,0 with heading 180)
    private static final double APRILTAG_X = -20.0; // AprilTag X position (was 15, now -20 after reset)
    private static final double APRILTAG_Y = 44.0; // AprilTag Y position (was 128, now 44 after reset)
    
    // Heading offset: if bot resets heading from 141° to 180°, offset is 39°


    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotor intake, flicker, rotator, theWheelOfTheOx;
    private DcMotorEx jollyCrusader;
    private Follower follower;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public void runOpMode() throws InterruptedException{
        follower = Constants.createFollower(hardwareMap);
        double desiredHeadingRad = Math.toRadians(141); // or whatever heading you want
        Pose startPoseWithHeading = new Pose(0, 0, desiredHeadingRad);
        follower.setStartingPose(startPoseWithHeading);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0,0.0328);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        jollyCrusader = hardwareMap.get(DcMotorEx.class, "launcher");
        jollyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jollyCrusader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jollyCrusader.setVelocity(0);
        //jollyCrusader.setDirection(DcMotorSimple.Direction.REVERSE);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator.setPower(1);

        intake = hardwareMap.get(DcMotor.class, "tree");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theWheelOfTheOx.setPower(0);
        theWheelOfTheOx.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }

        //dradle

        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()){
            time = runtime.time();
            boolean yPressed = gamepad1.dpad_down && !yLast;
            yLast = gamepad1.y;

            boolean aPressed = gamepad1.dpad_up && !aLast;
            aLast = gamepad1.a;
            
            boolean dpadUpPressed = gamepad1.dpad_up && !dpadUpLast;
            dpadUpLast = gamepad1.dpad_up;
            
            boolean dpadDownPressed = gamepad1.dpad_down && !dpadDownLast;
            dpadDownLast = gamepad1.dpad_down;
            boolean xPressed = gamepad2.x && !xLast;
            xLast = gamepad2.x;
            boolean rightBumperPressed = gamepad1.right_bumper && !rightBumperLast;
            rightBumperLast = gamepad1.right_bumper;

            if (rightBumperPressed) {
                rightBumperTimer.reset();
                rightBumperTimerStarted = true;
            }
//j
            if (!gamepad1.right_bumper) {
                rightBumperTimerStarted = false;
            }
//            if (gamepad1.x){
//                limelight.pipelineSwitch(1);
//            }
//            if (gamepad1.b){
//                limelight.pipelineSwitch(0);
//            }

            drive();



            //launcha
            if (aPressed){
                jollyCrusader.setVelocity(jollyCrusader.getVelocity()+20);
            }
            if (yPressed){
                jollyCrusader.setVelocity(jollyCrusader.getVelocity()-20);
            }




            //feed the flame ._.
            if (gamepad1.right_bumper){
                theWheelOfTheOx.setPower(1);
                intake.setPower(-1);
                gamepad1.rumble(100);


            }
            else if (gamepad1.left_bumper){
                theWheelOfTheOx.setPower(-1);
            }
            else {
                theWheelOfTheOx.setPower(0);
            }

            //intake
            sumOfTrigs = gamepad1.left_trigger-gamepad1.right_trigger;
            if (sumOfTrigs!=0){
                intake(sumOfTrigs);
            } else if (!gamepad1.right_bumper) {
                intake.setPower(0);
            }



            //rotator
            if (gamepad1.dpad_left){
                rotator.setTargetPosition(rotator.getCurrentPosition()-50);
            }
            else if (gamepad1.dpad_right){
                rotator.setTargetPosition(rotator.getCurrentPosition()+50);
            }
            else{
                rotator.setTargetPosition(rotator.getCurrentPosition());
            }

            // Localization adjustment with dpad_up/down (single press)
            if (dpadUpPressed || dpadDownPressed) {
                double localizationAngle = calculateAngleToAprilTag();
                
                // Limit localization angle to reasonable range (±180 degrees)
                if (localizationAngle > 180) localizationAngle -= 360;
                if (localizationAngle < -180) localizationAngle += 360;
                
                // Apply adjustment - use the actual angle to rotate towards the tag
                // dpad_up uses positive direction, dpad_down uses negative direction
                if (dpadUpPressed) {
                    // Rotate towards tag using positive adjustment
                    adjustRotatorWithLocalization(localizationAngle);
                } else if (dpadDownPressed) {
                    // Rotate towards tag using negative adjustment (reverse direction)
                    adjustRotatorWithLocalization(-localizationAngle);
                }
                
                localizationAdjustmentActive = true; // Flag to prevent limelight override
                
                telemetry.addData("Localization Angle", localizationAngle);
                telemetry.addData("Robot X", follower.getPose().getX());
                telemetry.addData("Robot Y", follower.getPose().getY());
                telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
            }
            
            // Clear flag when manual controls are used
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                localizationAdjustmentActive = false;
            }

            //Limelight calibration
            if (limelight != null) {
                LLResult ll = limelight.getLatestResult();
                double txDeg = 0.0; //horizontal deg
                double tyDeg = 0.0; //vertical deg
                double ta = 0.0;
                boolean llValid = false;
                if (ll != null) {
                    txDeg = ll.getTx();
                    tyDeg = ll.getTy();
                    ta = ll.getTa();
                    llValid = ll.isValid();

                    if (llValid) {
                        telemetry.addLine("Limelight Detecting Yes");
                        telemetry.addData("Ta", ta);
                        telemetry.addData("tx", txDeg);
                        telemetry.addData("ty", tyDeg);
                        // Don't adjust with limelight if using dpad_up/down for localization or manual controls
                        if (!gamepad1.dpad_right && !gamepad1.dpad_left && !localizationAdjustmentActive) {
                            double currentDistance = getDist(tyDeg);
                            // Use old adjustRotator function
                            adjustRotator(txDeg, currentDistance);
                        }
                    } else {
                        telemetry.addLine("Limelight Detecting No");
                        telemetry.addLine("no data");
                    }
                }
                double currentDistance = getDist(tyDeg);
                telemetry.addData("Distance", currentDistance);

                if (gamepad1.right_stick_button && currentDistance > 0){
                    targetVelocity = calcVelocity(currentDistance);
                    jollyCrusader.setVelocity(targetVelocity);
                    adjustHoodBasedOnDistance(currentDistance);
                    hasRumbledForVelocity = false;
                }

                if (targetVelocity > 0 && !hasRumbledForVelocity) {
                    double currentVelocity = jollyCrusader.getVelocity();
                    if (Math.abs(currentVelocity - targetVelocity) <= VELOCITY_LENIANCE) {
                        gamepad1.rumble(200); // Rumble for 200ms
                        hasRumbledForVelocity = true; // Only rumble once per target
                        telemetry.addLine("Velocity Reached!");
                    }
                }   


            } else {
                // Limelight is null - set rotator to zero if dpad_up/down not just pressed
                if (!dpadUpPressed && !dpadDownPressed) {
                    rotator.setTargetPosition(0);
                }
            }
            if (gamepad1.left_stick_button){
                jollyCrusader.setVelocity(0);
            }
//cool

            if (gamepad1.a){
                jollyCrusader.setVelocity(1400);
                hood.setPosition(0.0339);
            }
            if (gamepad1.x){
                hood.setPosition(hood.getPosition()-0.005);
            }
            if (gamepad1.b){
                hood.setPosition(hood.getPosition()+0.005);
            }


            telemetry.addData("jolly crusader velocity", jollyCrusader.getVelocity());
            telemetry.addData("rotator pos", rotator.getTargetPosition());
            telemetry.addData("hood pos", hood.getPosition());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();


        }


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
        leftRear.setPower(leftRearPower*driveMultiplier);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower*driveMultiplier);
    }
    /**
     * Calculate the angle from robot to AprilTag using localization
     * @return angle in degrees (relative to robot's front)
     */
    public void adjustRotator(double tx, double distance) {
        double fracOfFullCircum = Math.toRadians(tx) / (Math.PI);
        int adjustment = (int) (fracOfFullCircum * motor180Range);
        int offset = 14;
        if (distance > 200) {
            offset = 4;
        }
        int newPosition = rotator.getCurrentPosition() + adjustment - offset;
        rotator.setTargetPosition(newPosition);
    }
    public double calculateAngleToAprilTag() {
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();
        
        // Account for heading reset: if heading was reset from 141° to 180°, add offset
        double deltaX = APRILTAG_X - robotX;
        double deltaY = APRILTAG_Y - robotY;

        // Calculate absolute angle to AprilTag (in radians)
        // atan2(y, x) gives angle from positive x-axis
        double angleToTag = Math.atan2(deltaY, deltaX);

        // Calculate relative angle (how much to rotate from robot's current heading)
        // This is the angle the turret needs to point relative to robot's front
        // Use adjusted heading to account for the reset
        double relativeAngle = angleToTag - robotHeading;

        // Normalize to [-PI, PI]
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

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



    public void adjustRotatorWithLocalization(double localizationAngleDeg) {
        int currentPos = rotator.getTargetPosition();

        // Check if rotator has reached 225 degree limits on either side (encoder-based)
        boolean atPositiveHalfway = currentPos >= HALFWAY_POSITIVE;
        boolean atNegativeHalfway = currentPos <= HALFWAY_NEGATIVE;

        // Apply deadband - don't adjust if angle is very small
        if (Math.abs(localizationAngleDeg) < 2.0) {
            rotator.setTargetPosition(currentPos);
            return;
        }


        double fracOfSemiCircum = Math.toRadians(localizationAngleDeg) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);

        // Determine adjustment direction based on encoder position
        if (atPositiveHalfway) {
            // Hit positive 225 degree limit (+787 ticks), reverse direction (use negative adjustment)
            adjustment = -Math.abs(adjustment);
            telemetry.addData("Status", "+");
        } else if (atNegativeHalfway) {
            // Hit negative 225 degree limit (-787 ticks), reverse direction (use positive adjustment)
            adjustment = Math.abs(adjustment);
            telemetry.addData("Status", "-");
        } else {
            adjustment = -adjustment;
        }

        // Rate limiting removed - adjustment now uses full calculated value

        int offset = 14;

        int newPosition = currentPos + adjustment - offset;
        rotator.setTargetPosition(applyHardLimiter(newPosition));

        telemetry.addData("Using", "Localization Only");
        telemetry.addData("Current Pos", currentPos);
        telemetry.addData("Loc Angle Deg", localizationAngleDeg);
        telemetry.addData("Adjustment Ticks", adjustment);
        telemetry.addData("New Pos", newPosition);
    }

    public double getDist(double tyDeg) {
        double tyRad = Math.abs(Math.toRadians(tyDeg+limelightUpAngle));
        double dist = y / Math.tan(tyRad);
        double realDist = 0.55*dist+40.3;
        telemetry.addData("angle", Math.toDegrees(tyRad));
        telemetry.addData("fakeDist", dist);
        telemetry.addData("realDist", realDist);
        return realDist;
    }
    public double calcVelocity(double dist) {
        double velocity;
        if (dist < FIRST_DISTANCE_THRESHOLD) {
            velocity = 4.94*dist + 1008;
        } else if (dist < SECOND_DISTANCE_THRESHOLD){
            velocity = 4.22*dist + 1129;
        }
        else{
            velocity = 16.66*dist - 1420;
        }
        return velocity;
    }
    public void intake(double intakePower){
        intake.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-0.4);
        }
    }



    public void adjustHoodBasedOnDistance(double dist) {
        if (dist < FIRST_DISTANCE_THRESHOLD) {
            hood.setPosition(CLOSE_HOOD_POSITION);
        } else if (dist < SECOND_DISTANCE_THRESHOLD){
            hood.setPosition(MID_HOOD_POSITION);
        }
        else{
            hood.setPosition(FAR_HOOD_POSITION);
        }
    }

    /**
     * Applies hard limiter to rotator target position.
     * If target exceeds limits, calculates position going the other way instead.
     * @param targetPosition The desired target position
     * @return The clamped position that respects hard limits (reversed if needed)
     */
    private int applyHardLimiter(int targetPosition) {
        int currentPos = rotator.getCurrentPosition();
        
        // Check if target would exceed positive limit
        if (targetPosition > HARD_LIMIT_POSITIVE) {
            // Calculate how far past the limit we would go
            int excess = targetPosition - HARD_LIMIT_POSITIVE;
            // Reverse direction: go from positive limit towards negative
            int reversedPosition = HARD_LIMIT_POSITIVE - excess;
            // Make sure we don't go past negative limit either
            if (reversedPosition < HARD_LIMIT_NEGATIVE) {
                reversedPosition = HARD_LIMIT_NEGATIVE;
            }
            telemetry.addData("Hard Limiter", "Positive limit hit, reversing");
            return reversedPosition;
        }
        
        // Check if target would exceed negative limit
        if (targetPosition < HARD_LIMIT_NEGATIVE) {
            // Calculate how far past the limit we would go
            int excess = HARD_LIMIT_NEGATIVE - targetPosition;
            // Reverse direction: go from negative limit towards positive
            int reversedPosition = HARD_LIMIT_NEGATIVE + excess;
            // Make sure we don't go past positive limit either
            if (reversedPosition > HARD_LIMIT_POSITIVE) {
                reversedPosition = HARD_LIMIT_POSITIVE;
            }
            telemetry.addData("Hard Limiter", "Negative limit hit, reversing");
            return reversedPosition;
        }
        
        // Target is within limits, return as-is
        return targetPosition;
    }

}