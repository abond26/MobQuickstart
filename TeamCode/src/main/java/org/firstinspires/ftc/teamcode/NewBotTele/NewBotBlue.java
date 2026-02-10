package org.firstinspires.ftc.teamcode.NewBotTele;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp

public class NewBotBlue extends LinearOpMode {

    double newTime;
    double time;
    double F = 12.35;
    double P = 132.5000;


    //282
    //-301
    double sumOfTrigs;
    boolean yLast = false;
    boolean aLast =false;
    boolean xLast = false;
    boolean rightBumperLast = false;
    boolean resetLocLast = false; // For localization reset button
    ElapsedTime rightBumperTimer = new ElapsedTime();
    boolean rightBumperTimerStarted = false;
    public static final int ROTATOR_MIN_TICKS = -1096;
    public static final int ROTATOR_MAX_TICKS = 1096;
    private static final double HOOD_MOVE_DELAY_SECONDS = 0.5; // Time to hold button before hood moves
    int motor180Range = 1250;
    int limelightUpAngle = 20;
    private int limeHeight = 35;
    private int tagHeight = 75;
    private int y = tagHeight - limeHeight;
    public static double driveMultiplier = 1;
    private double lastTurretAngleDeg = 0;

    private Limelight3A limelight;
    public ElapsedTime runtime = new ElapsedTime();
    private Servo hood,blocker;

    // Distance threshold for hood adjustment (matching lookup table zones)
    private static final double FIRST_DISTANCE_THRESHOLD = 145.0;  // Zone 1: Close (< 145)
    private static final double SECOND_DISTANCE_THRESHOLD = 200.0; // Zone 2: Mid (145-200), Zone 3: Far (>= 200)
    private static final double CLOSE_HOOD_POSITION = 1; // Hood position for close shots
    private static final double MID_HOOD_POSITION = 0.5;
    private static final double FAR_HOOD_POSITION = 0; // Hood position for far shots
    private Pose startPose = new Pose(0, 0, 0);
    Pose bluePos = new Pose(11, 137, 0);
    Pose redPos = new Pose(133, 137, 0);
    Pose target = bluePos;
    
    // AprilTag positions for relocalization (from PoseStorage)
    private static final double BLUE_APRILTAG_X = 24.4;
    private static final double BLUE_APRILTAG_Y = 126.7;
    private static final double RED_APRILTAG_X = 117.6;
    private static final double RED_APRILTAG_Y = 130.0;
    
    // Relocalization settings
    private static final double RELOCALIZATION_DISTANCE_THRESHOLD = 120.0; // Only relocalize if within 120 inches
    private static final double RELOCALIZATION_ANGLE_THRESHOLD = 30.0; // Only relocalize if angle difference < 30°
    private boolean relocalizationEnabled = true;
    private DcMotor intake, flicker, rotator, theWheelOfTheOx;
    private DcMotorEx jollyCrusader;
    private Follower follower;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public void runOpMode() throws InterruptedException{
        follower = Constants.createFollower(hardwareMap);
        startPose = PoseStorage.loadPose(new Pose(24.4, 126.7, Math.toRadians(143)));
        follower.setStartingPose(startPose);
        
        // Update follower once before waitForStart for proper initialization
        follower.update();

        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0,0.0761);
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0, 0.4);
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
        jollyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jollyCrusader.setDirection(DcMotorSimple.Direction.FORWARD);
        jollyCrusader.setVelocity(0);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        jollyCrusader.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //jollyCrusader.setDirection(DcMotorSimple.Direction.REVERSE);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(0.5); // Reduced from 1.0 to 0.5 to prevent belt skipping

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
            follower.update();
            Pose robotPose = follower.getPose();
            double x = robotPose.getX();
            double y = robotPose.getY();
            double headingDeg = Math.toDegrees(robotPose.getHeading());
            double turretAngleDeg = alignTurret(x, y, headingDeg, target);
            time = runtime.time();
            boolean yPressed = gamepad1.dpad_down && !yLast;
            yLast = gamepad1.dpad_down;

            boolean aPressed = gamepad1.dpad_up && !aLast;
            aLast = gamepad1.dpad_up;
            
            // Localization reset button (gamepad2.y) - reinitialize pose when localization breaks
            // Uses Limelight relocalization if AprilTag is detected, otherwise falls back to manual reset
            boolean resetLocPressed = gamepad2.y && !resetLocLast;
            resetLocLast = gamepad2.y;
            if (resetLocPressed) {
                boolean relocalized = false;
                
                // Try to relocalize using Limelight if available and seeing AprilTag
                if (limelight != null) {
                    LLResult ll = limelight.getLatestResult();
                    if (ll != null && ll.isValid()) {
                        double txDeg = ll.getTx();
                        double tyDeg = ll.getTy();
                        double dist = getDist(tyDeg);
                        
                        if (dist > 0 && dist < RELOCALIZATION_DISTANCE_THRESHOLD) {
                            // Use Limelight relocalization
                            relocalizeFromAprilTag(txDeg, tyDeg, dist);
                            relocalized = true;
                            telemetry.addLine("LOCALIZATION RESET via Limelight!");
                        }
                    }
                }
                
                // Fallback: Manual reset if Limelight not available or not seeing tag
                if (!relocalized) {
                    // Get current robot heading (how the bot is actually facing)
                    Pose currentPose = follower.getPose();
                    double currentHeading = currentPose.getHeading();
                    
                    // Reset X, Y to known position (from PoseStorage or default), but keep current heading
                    Pose defaultPose = PoseStorage.loadPose(new Pose(24.4, 126.7, Math.toRadians(143)));
                    Pose resetPose = new Pose(defaultPose.getX(), defaultPose.getY(), currentHeading);
                    
                    follower.setStartingPose(resetPose);
                    follower.update(); // Update once to initialize
                    telemetry.addLine("LOCALIZATION RESET (Manual - No Limelight)!");
                    telemetry.addData("Reset X", resetPose.getX());
                    telemetry.addData("Reset Y", resetPose.getY());
                    telemetry.addData("Reset Heading (from bot)", Math.toDegrees(resetPose.getHeading()));
                }
            }
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
                jollyCrusader.setVelocity(jollyCrusader.getVelocity()+30);
            }
            if (yPressed){
                jollyCrusader.setVelocity(jollyCrusader.getVelocity()-30);
            }




            //feed the flame ._.
            if (gamepad1.right_bumper){
                blocker.setPosition(1);
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
                //theWheelOfTheOx.setPower(-1);
                blocker.setPosition(0);
                intake(sumOfTrigs);
            } else if (!gamepad1.right_bumper) {
                intake.setPower(0);
            }



            //rotator
            if (gamepad1.dpad_left){
                rotator.setTargetPosition(rotator.getCurrentPosition()-100);
            }
            else if (gamepad1.dpad_right){
                rotator.setTargetPosition(rotator.getCurrentPosition()+100);
            }
            else{
                rotator.setTargetPosition(rotator.getCurrentPosition());
            }

            // Rotator control: Always use localization as base, add Limelight fine-tuning when available
            double finalTurretAngle = turretAngleDeg; // Start with localization angle
            double tyDeg = 0.0; // For velocity/hood adjustment
            boolean llValid = false;
            
            //Limelight calibration
            if (limelight != null) {
                LLResult ll = limelight.getLatestResult();
                double txDeg = 0.0; //horizontal deg
                double ta = 0.0;
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
                        
                        // Calculate distance to AprilTag
                        double dist = getDist(tyDeg);
                        
                        // Relocalization: Use AprilTag to correct pose estimate
                        if (relocalizationEnabled && dist > 0 && dist < RELOCALIZATION_DISTANCE_THRESHOLD) {
                            relocalizeFromAprilTag(txDeg, tyDeg, dist);
                        }
                        
                        // Combine localization base angle with Limelight fine-tuning
                        // Only adjust if not manually controlling with dpad
                        if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                            // Add Limelight tx adjustment to localization angle
                            // Negate tx for blue side to fix inverted localization
                            finalTurretAngle = turretAngleDeg - txDeg;
                            // Normalize to [-180, 180] - rotator can only go 180° each direction
                            while (finalTurretAngle > 180) finalTurretAngle -= 360;
                            while (finalTurretAngle < -180) finalTurretAngle += 360;
                        }
                    } else {
                        telemetry.addLine("Limelight Detecting No");
                        telemetry.addLine("no data");
                        // Use localization only when Limelight not available
                    }
                }
                // Auto-adjust velocity and hood based on dist using lookup table
                // ONLY when Limelight is valid and sees a target
                if (llValid && tyDeg != 0) {
                    // Use getDist() which now returns dist (fakeDist) - FIXED: use tyDeg not txDeg!
                    double dist = getDist(tyDeg);
                    telemetry.addData("Distance (dist)", dist);

                    if (dist > 0 && !Double.isNaN(dist) && !Double.isInfinite(dist)) {
                        // Auto-adjust velocity from lookup table using dist
                        double autoVelocity = VelocityLookupTable.getVelocity(dist);
                        jollyCrusader.setVelocity(autoVelocity);
                        telemetry.addData("Auto Velocity", autoVelocity);

                        // Auto-adjust hood based on zone using dist
                        adjustHoodBasedOnDistance(dist);
                    }
                }

                // Manual override buttons (still work if needed - these override auto velocity)
                if (gamepad1.right_stick_button){
                    jollyCrusader.setVelocity(2300);
                }
//fasd

            }
            
            // Set rotator to final calculated angle (localization + Limelight adjustment)
            if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                setRotatorToTurretAngle(finalTurretAngle);
                telemetry.addData("Final Turret Angle", finalTurretAngle);
                telemetry.addData("Base Loc Angle", turretAngleDeg);
            }
            if (gamepad1.left_stick_button){
                jollyCrusader.setVelocity(1200);
            }
//cool
            if(gamepad1.a)
            {
                hood.setPosition(1);
            }



            if (gamepad1.x){
                hood.setPosition(hood.getPosition()-0.25);
            }
            if (gamepad1.b){
                hood.setPosition(hood.getPosition()+0.25);
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
    private double alignTurret(double x, double y, double headingDeg, Pose target) {
        double dx = target.getX() - x;
        double dy = target.getY() - y;
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        // Fix inverted localization for blue side - negate the angle
        double turretAngle = -(angleToGoal - headingDeg);
        
        // Normalize to [-180, 180]
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;
        
        // Handle angle wrapping to prevent 180° flips
        double diff = turretAngle - lastTurretAngleDeg;
        if (diff > 90) {
            turretAngle -= 360;
        } else if (diff < -90) {
            turretAngle += 360;
        }
        
        // Normalize again after wrapping
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;
        
        lastTurretAngleDeg = turretAngle;
        return turretAngle;
    }
    private final int ROTATOR_ZERO_TICKS = 0;  // tick position when t8[888[urret faces forward; calibrate if needed

    public void setRotatorToTurretAngle(double turretAngleDeg) {
        double fracOf180 = Math.toRadians(turretAngleDeg) / Math.PI;
        int targetTicks = ROTATOR_ZERO_TICKS + (int) (fracOf180 * motor180Range);
        // Optional: clamp to physical limits (e.g. ±270°)
        rotator.setTargetPosition(targetTicks);
    }
    public void adjustRotator(double tx, double distance) {
        double fracOfFullCircum = Math.toRadians(tx) / (Math.PI);
        int adjustment = (int) (fracOfFullCircum * motor180Range);
        int offset = 0;
        if (distance > 200) {
            offset = 0;
        }
        int newPosition = rotator.getCurrentPosition() + adjustment - offset;
        rotator.setTargetPosition(newPosition);
    }

    public double getDist(double tyDeg) {
        double tyRad = Math.abs(Math.toRadians(tyDeg+limelightUpAngle));
        double dist = y / Math.tan(tyRad);
        //double realDist = 0.55*dist+40.3;
        telemetry.addData("angle", Math.toDegrees(tyRad));
        telemetry.addData("fakeDist", dist);
        //telemetry.addData("realDist", realDist);
        return dist;
    }
    /**
     * Calculates velocity using lookup table for more consistent results.
     * The lookup table uses linear interpolation between pre-computed values,
     * eliminating formula calculation errors and providing more predictable behavior.
     *
     */
    //adjklfajflkaj
    public void intake(double intakePower){
        intake.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            //theWheelOfTheOx.setPower(0.4);
        }
    }



    /**
     * Automatically adjusts hood position based on distance.
     * Uses the same 3 zones as the velocity lookup table:
     * - Zone 1 (Close): < 145 inches → CLOSE_HOOD_POSITION
     * - Zone 2 (Mid): 145-200 inches → MID_HOOD_POSITION
     * - Zone 3 (Far): >= 200 inches → FAR_HOOD_POSITION
     */
    public void adjustHoodBasedOnDistance(double dist) {
        int zone = VelocityLookupTable.getZone(dist);
        if (zone == 1) {
            // Zone 1: Close range (< 140)
            hood.setPosition(CLOSE_HOOD_POSITION);
        } else if (zone == 2) {
            // Zone 2: Mid range (140-200)
            hood.setPosition(MID_HOOD_POSITION);
        } else {
            // Zone 3: Far range (>= 200)
            hood.setPosition(FAR_HOOD_POSITION);
        }
    }

    /**
     * Relocalizes robot pose using AprilTag detection from Limelight.
     * Calculates robot position based on known AprilTag position, distance, and angles.
     * Only corrects pose if within reasonable distance and angle thresholds.
     */
    private void relocalizeFromAprilTag(double txDeg, double tyDeg, double distance) {
        // Get AprilTag position (blue side for NewBotBlue)
        double aprilTagX = BLUE_APRILTAG_X;
        double aprilTagY = BLUE_APRILTAG_Y;
        
        // Calculate angle from robot to AprilTag
        // tx is horizontal offset, so we need to account for robot heading
        Pose currentPose = follower.getPose();
        double currentHeading = Math.toDegrees(currentPose.getHeading());
        
        // Calculate angle to AprilTag from current pose estimate
        double dx = aprilTagX - currentPose.getX();
        double dy = aprilTagY - currentPose.getY();
        double angleToTag = Math.toDegrees(Math.atan2(dy, dx));
        
        // Calculate what the angle should be based on Limelight tx
        // tx is the horizontal offset: positive = tag is to the right, negative = tag is to the left
        double correctedAngleToTag = currentHeading + txDeg;
        
        // Calculate robot position from AprilTag
        // Robot is at: AprilTag position - (distance * direction vector)
        double angleRad = Math.toRadians(correctedAngleToTag);
        double robotX = aprilTagX - distance * Math.cos(angleRad);
        double robotY = aprilTagY - distance * Math.sin(angleRad);
        
        // Calculate corrected heading
        // Heading = angle to tag - tx offset (accounting for camera orientation)
        double correctedHeading = correctedAngleToTag - txDeg;
        
        // Only update if the correction is reasonable (not a huge jump)
        double poseDiff = Math.sqrt(
            Math.pow(robotX - currentPose.getX(), 2) + 
            Math.pow(robotY - currentPose.getY(), 2)
        );
        double headingDiff = Math.abs(Math.toDegrees(correctedHeading) - currentHeading);
        
        if (poseDiff < 24.0 && headingDiff < RELOCALIZATION_ANGLE_THRESHOLD) {
            // Update pose estimate
            Pose correctedPose = new Pose(robotX, robotY, Math.toRadians(correctedHeading));
            follower.setPose(correctedPose);
            
            telemetry.addLine("RELOCALIZED from AprilTag!");
            telemetry.addData("Corrected X", robotX);
            telemetry.addData("Corrected Y", robotY);
            telemetry.addData("Corrected Heading", Math.toDegrees(correctedHeading));
        } else {
            telemetry.addLine("Relocalization skipped (too large correction)");
            telemetry.addData("Pose diff", poseDiff);
            telemetry.addData("Heading diff", headingDiff);
        }
    }

}