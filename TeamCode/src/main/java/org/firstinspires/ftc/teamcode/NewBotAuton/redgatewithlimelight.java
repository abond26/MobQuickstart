package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "Reliable red gate limelight?", group = "limelight new bot")
public class redgatewithlimelight extends OpMode {
    private int rotatorStartPosition=0;
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;

    // Flags to prevent path oscillation - ensure paths are only called once per state
    private boolean shoot2Started = false;
    private boolean shoot3Started = false;
    private boolean shoot4Started = false;
    private boolean shoot5Started = false;
    private boolean goAwayFromGateStarted = false;
    private boolean goTowardsGateStarted = false;
    private boolean opengateStarted = false;
    private boolean parkingStarted = false;
    private boolean collectionStarted = false;
    private boolean beginGateCollectionStarted = false;
    private boolean gateCollectionStarted = false;
    private boolean beginGateCollectionAgainStarted = false;
    private boolean gateCollectionAgainStarted = false;

    private Servo hood;
    private int limeHeight = 33;
    private int offset = 28;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 1; // tune this

    // Hood adjustment constants (from TesterinoRed)
    private static final double FIRST_DISTANCE_THRESHOLD = 140.0;
    private static final double SECOND_DISTANCE_THRESHOLD = 200.0;
    private static final double CLOSE_HOOD_POSITION = 0.0309; // Hood position for close shots
    private static final double MID_HOOD_POSITION = 0.18 + 0.0167;
    private static final double FAR_HOOD_POSITION = 0.16 + 0.0129; // Hood position for far shots

    private int y = tagHeight - limeHeight;
    //Rotator var
    int motor180Range = 630; // Match TesterinoRed
    int limelightUpAngle = 20; // Match TesterinoRed
    private int vMultiplier = 9;
    private Limelight3A limelight;
    
    // Rotator safeguards to prevent erratic movement
    private static final double TX_DEADBAND = 0.3; // Don't adjust if tx is less than 0.3 degrees (more sensitive)
    private static final int MIN_MOVEMENT_THRESHOLD = 10; // Don't adjust if movement is less than 10 ticks
    private static final int MAX_ADJUSTMENT_PER_CALL = 100; // Clamp adjustment to max 100 ticks per call
    private static final int ROTATOR_SETTLE_THRESHOLD = 20; // Only adjust when within 20 ticks of target
    private int desiredRotatorPosition = 0; // Track desired position separately to prevent cumulative errors
    private int adjustmentLoopCounter = 0;
    private static final int ADJUSTMENT_RATE_LIMIT = 3; // Only adjust every 3 loops (rate limiting)
    private com.qualcomm.robotcore.util.ElapsedTime lastAdjustmentTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    
    // Rotator physical limits to prevent 360-degree rotations and damage
    private static final int DEGREES_270_TICKS = 945; // 270 degrees in ticks (630 * 1.5)
    private static final int MAX_ROTATOR_POSITION = DEGREES_270_TICKS; // Physical limit
    private static final int MIN_ROTATOR_POSITION = -DEGREES_270_TICKS; // Physical limit

    // Store last valid limelight values for fallback
    private double lastValidTx = 0.0;
    private double lastValidTy = 0.0;
    private double lastValidDistance = 0.0;
    private boolean hasValidLimelightData = false;

    private DcMotor leftFront, leftRear, rightFront, rightRear;


    private DcMotorEx launcher;
    private DcMotor tree, theWheelOfTheOx, rotator;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        start,
        actuallyshoot1,
        gotocollect,
        collection,
        goAwayFromGate,
        goTowardsGate,
        opengate,
        shoot,
        collectAgain,
        collectAgainEnd,


        shootAgain,

        collectAgainAgain,


        collectAgainAgainEnd,

        shootAgainAgain,

        collectAgainAgainAgain,

        collectAgainAgainAgainEnd,

        shootAgainAgainAgain,
        parklol,


        done,

        VERYYYY_THIRD_INTAKE,

        THIRD_SHOT_PREP,

        PAUSE3,

        SHOT_3,

        JACK_OFF,
        beginGateCollection,
        GateCollection,
        beginGateCollectionAgain,
        GateCollectionAgain,

    }

    PathState pathState;
    private final Pose startPose = new Pose(118, 132.2, Math.toRadians(36.5));
    private final Pose shootPose1 = new Pose(88, 92, Math.toRadians(48.5));
    private final Pose collect1thingstart = new Pose(88, 57.5, Math.toRadians(0));
    private final Pose collect1thing = new Pose(125, 57.5, Math.toRadians(0));
    private final Pose shootPose2 = new Pose( 87, 84, Math.toRadians(48.5));
    private final Pose gateCollect1 = new Pose( 130, 61, Math.toRadians(25));
    private final Pose inBetween1 = new Pose(100, 61, Math.toRadians(25));
    private final Pose shootBall3 = new Pose(87, 84, Math.toRadians(51));
    private final Pose inBetween2 = new Pose(100, 61, Math.toRadians(25));

    private final Pose gateCollect2 = new Pose( 130, 61, Math.toRadians(25));
    private final Pose shootBall4 = new Pose(87, 84, Math.toRadians(51));
    private final Pose collect3start=new Pose(95, 35, Math.toRadians(0));

    //
    private final Pose collect3end = new Pose(127, 35, Math.toRadians(0));
    private final Pose park = new Pose(103, 84, Math.toRadians(46));



    private PathChain shoot1, goToCollect1, collect1, shoot2,InBetween1, InBetween2, GateCollect1, GateCollect2, shoot3, awayfromGate, goToCollect3, collect3, shoot4, goToGate, openGate, goToCollect4, collect4, shoot5, parking;

    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        goToCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, collect1thingstart))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), collect1thingstart.getHeading())
                .build();

        collect1 = follower.pathBuilder()
                .addPath(new BezierLine(collect1thingstart, collect1thing))
                .setLinearHeadingInterpolation(collect1thingstart.getHeading(), collect1thing.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1thing, shootPose2))
                .setLinearHeadingInterpolation(collect1thing.getHeading(), shootPose2.getHeading())
                .build();
        InBetween1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, inBetween1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), inBetween1.getHeading())
                .build();

        GateCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(inBetween1, gateCollect1))
                .setLinearHeadingInterpolation(inBetween1.getHeading(), gateCollect1.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(gateCollect1, shootBall3))
                .setLinearHeadingInterpolation(gateCollect1.getHeading(), shootBall3.getHeading())
                .build();
        InBetween2 = follower.pathBuilder()
                .addPath(new BezierLine(shootBall3, inBetween2))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), inBetween2.getHeading())
                .build();
        GateCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(inBetween2, gateCollect2))
                .setLinearHeadingInterpolation(inBetween2.getHeading(), gateCollect2.getHeading())
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(gateCollect2, shootBall4))
                .setLinearHeadingInterpolation(gateCollect2.getHeading(), shootBall4.getHeading())
                .build();

//        goToCollect3 = follower.pathBuilder()
//                .addPath(new BezierLine(shootBall3, collect3start))
//                .setLinearHeadingInterpolation(shootBall3.getHeading(), collect3start.getHeading())
//                .build();
//
//        collect3 = follower.pathBuilder()
//                .addPath(new BezierLine(collect3start, collect3end))
//                .setLinearHeadingInterpolation(collect3start.getHeading(), collect3end.getHeading())
//                .build();
//
//
//        shoot4 = follower.pathBuilder()
//                .addPath(new BezierLine(collect3end, shootBall4))
//                .setLinearHeadingInterpolation(collect3end.getHeading(), shootBall4.getHeading())
//                .build();
//
//        parking=follower.pathBuilder()
//                .addPath(new BezierLine(shootBall4, park))
//                .setLinearHeadingInterpolation(shootBall4.getHeading(), park.getHeading())
//                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                tree.setPower(0.125);
                // Limelight will set velocity and hood in actuallyshoot1 state
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(shoot1);
                setPathState(redgatewithlimelight.PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                // Sync desired position with current position when entering shooting state
                if (!shoot2Started) {
                    desiredRotatorPosition = rotator.getCurrentPosition();
                }
                // Continuously adjust based on limelight during shooting
                updateLimelightAdjustments();
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>4){
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(-1);
                    if (pathTimer.getElapsedTimeSeconds()>5) {
                        setPathState(redgatewithlimelight.PathState.collection);
                    }
                }
                break;
            case gotocollect:
                //rotator.setTargetPosition(rotatorStartPosition);
                if(!follower.isBusy())
                {
                    //rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    follower.followPath(goToCollect1);
                    setPathState(redgatewithlimelight.PathState.collection);
                }
                break;


            case collection:
                if (!follower.isBusy() && !collectionStarted) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    theWheelOfTheOx.setPower(0.8);
                    tree.setPower(1);
                    follower.followPath(collect1);
                    collectionStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && collectionStarted) {
                    setPathState((PathState.shoot));
                }
                break;
            case shoot:
                // Sync desired position with current position when entering shooting state
                if (!shoot2Started) {
                    desiredRotatorPosition = rotator.getCurrentPosition();
                }
                // Continuously adjust based on limelight during shooting
                updateLimelightAdjustments();
                if (!follower.isBusy() && !shoot2Started) {
                    follower.followPath(shoot2);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot2Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot2Started) {
                    if(pathTimer.getElapsedTimeSeconds()>2) {
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>5) {
                        setPathState((PathState.beginGateCollection));
                    }
                }
                break;

            case beginGateCollection:
                //rotator.setTargetPosition(rotatorStartPosition);
                if (!follower.isBusy() && !beginGateCollectionStarted) {
                    //rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    follower.followPath(InBetween1);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    beginGateCollectionStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && beginGateCollectionStarted) {
                    setPathState((PathState.GateCollection));
                }
                break;
            case GateCollection:
                if (!follower.isBusy() && !gateCollectionStarted) {
                    follower.followPath(GateCollect1);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(0.8);
                    gateCollectionStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && gateCollectionStarted && pathTimer.getElapsedTimeSeconds()>3.5) {
                    setPathState((redgatewithlimelight.PathState.shootAgain));
                }
                break;
            case shootAgain:
                // Sync desired position with current position when entering shooting state
                if (!shoot3Started) {
                    desiredRotatorPosition = rotator.getCurrentPosition();
                }
                // Continuously adjust based on limelight during shooting
                updateLimelightAdjustments();
                if (!follower.isBusy() && !shoot3Started) {
                    follower.followPath(shoot3);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot3Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot3Started) {
                    if(pathTimer.getElapsedTimeSeconds()>3.5) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>4.5)
                    {
                        setPathState((PathState.beginGateCollectionAgain));
                    }
                }
                break;
            case beginGateCollectionAgain:
                if (!follower.isBusy() && !beginGateCollectionAgainStarted) {
                    //rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    follower.followPath(InBetween2);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    beginGateCollectionAgainStarted = true; // Mark as started to prevent calling again
                    //rotator.setTargetPosition(rotatorStartPosition);
                }
                if (!follower.isBusy() && beginGateCollectionAgainStarted) {
                    setPathState((PathState.GateCollectionAgain));
                }
                break;
            case GateCollectionAgain:
                if (!follower.isBusy() && !gateCollectionAgainStarted) {
                    follower.followPath(GateCollect2);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(0.8);
                    gateCollectionAgainStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && gateCollectionAgainStarted && pathTimer.getElapsedTimeSeconds()>3.5) {
                    setPathState((redgatewithlimelight.PathState.shootAgainAgain));
                }
                break;
            case shootAgainAgain:
                // Sync desired position with current position when entering shooting state
                if (!shoot4Started) {
                    desiredRotatorPosition = rotator.getCurrentPosition();
                }
                // Continuously adjust based on limelight during shooting
                updateLimelightAdjustments();
                if (!follower.isBusy() && !shoot4Started) {
                    follower.followPath(shoot4);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot4Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot4Started) {
                    if(pathTimer.getElapsedTimeSeconds()>3.5) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>4.5)
                    {
                        setPathState((redgatewithlimelight.PathState.done));
                    }
                }
                break;
            case done:
                // Save final pose and calculate rotator position to face goal

                // Calculate and set rotator to face the goal (red side = true, motor180Range = 910, offset = 28)

                // Save rotator position for teleop
                break;

        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        // Reset flags when state changes to allow paths to be called again in new state
        shoot2Started = false;
        shoot3Started = false;
        shoot4Started = false;
        shoot5Started = false;
        goAwayFromGateStarted = false;
        goTowardsGateStarted = false;
        opengateStarted = false;
        parkingStarted = false;
        collectionStarted = false;
        beginGateCollectionStarted = false;
        gateCollectionStarted = false;
        beginGateCollectionAgainStarted = false;
        gateCollectionAgainStarted = false;
    }

    @Override
    public void init() {
        pathState = PathState.start;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = ConstantsNewBot.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);


        tree = hardwareMap.get(DcMotor.class, "tree");
        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(0.0119);
        hood.scaleRange(0,0.025);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorStartPosition=0;
        desiredRotatorPosition = rotatorStartPosition; // Initialize desired position
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double P = 132.5;
        double I = 0;
        double D = 0;
        double F = 12.35;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        
        // Initialize Limelight (red side uses pipeline 0)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(0);
            limelight.start();
        }
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        // Continuously save pose so it's saved even if autonomous ends early
        PoseStorage.savePose(follower.getPose());

        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("jolly crusader velocity", launcher.getVelocity());
    }

    public double getDist(double tyDeg) {
        // Use corrected distance formula from TesterinoRed
        double tyRad = Math.abs(Math.toRadians(tyDeg + limelightUpAngle));
        double dist = y / Math.tan(tyRad);
        double realDist = 0.55 * dist + 40.3; // Correction formula from TesterinoRed
        return realDist;
    }
    public double calcVelocity(double dist) {
        // Use piecewise linear formula from TesterinoRed
        double velocity;
        if (dist < FIRST_DISTANCE_THRESHOLD) {
            velocity = 4.94 * dist + 1008;
        } else if (dist < SECOND_DISTANCE_THRESHOLD) {
            velocity = 4.22 * dist + 1129;
        } else {
            velocity = 16.66 * dist - 1420;
        }
        return velocity;
    }

    public void intake(double intakePower){
        tree.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-0.3);
        }
    }
    // Rotator adjustment from TesterinoRed - uses distance-based offset with safeguards
    public void adjustRotator(double tx, double distance) {
        // CRITICAL SAFEGUARDS: Prevent 180-degree wrong-direction rotations and erratic movement
        
        // 1. Deadband - don't adjust if tx is too small
        if (Math.abs(tx) < TX_DEADBAND) {
            return; // Already aligned, don't adjust
        }
        
        // 2. Rate limiting - only adjust every N loops or every 150ms
        adjustmentLoopCounter++;
        boolean shouldAdjustNow = false;
        if (adjustmentLoopCounter >= ADJUSTMENT_RATE_LIMIT || 
            lastAdjustmentTimer.seconds() > 0.15) {
            shouldAdjustNow = true;
            adjustmentLoopCounter = 0;
            lastAdjustmentTimer.reset();
        }
        if (!shouldAdjustNow) {
            return; // Skip this adjustment cycle
        }
        
        // 3. Only adjust when rotator is settled (close to target) to prevent cumulative errors
        int currentPos = rotator.getCurrentPosition();
        int currentTarget = rotator.getTargetPosition();
        if (Math.abs(currentPos - currentTarget) > ROTATOR_SETTLE_THRESHOLD) {
            return; // Rotator still moving, wait for it to settle
        }
        
        // 4. When rotator settles, sync desired position to actual position
        // This prevents offset accumulation that causes drift
        desiredRotatorPosition = currentPos;
        
        // 5. Match TesterinoRed.java calculation exactly
        double fracOfFullCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfFullCircum * motor180Range);
        
        // 6. Clamp adjustment per call to prevent large jumps
        if (adjustment > MAX_ADJUSTMENT_PER_CALL) {
            adjustment = MAX_ADJUSTMENT_PER_CALL;
        }
        if (adjustment < -MAX_ADJUSTMENT_PER_CALL) {
            adjustment = -MAX_ADJUSTMENT_PER_CALL;
        }
        
        // 7. Minimum movement threshold - don't adjust if movement is too small
        if (Math.abs(adjustment) < MIN_MOVEMENT_THRESHOLD) {
            return; // Movement too small, skip adjustment
        }
        
        // 8. Distance-based offset (from TesterinoRed)
        int offset = -10;
        if (distance < 120) {
            offset = -10;
        } else if (distance > 180) {
            offset = -15;
        }
        
        // 9. Calculate from CURRENT position (like TesterinoRed) to prevent offset accumulation
        int newPosition = currentPos + adjustment + offset;
        
        // 10. CRITICAL: Clamp final position to physical limits to prevent 360-degree rotations
        // This ensures the rotator never goes beyond ±270 degrees
        if (newPosition > MAX_ROTATOR_POSITION) {
            newPosition = MAX_ROTATOR_POSITION;
        }
        if (newPosition < MIN_ROTATOR_POSITION) {
            newPosition = MIN_ROTATOR_POSITION;
        }
        
        // 11. Update desired position and set target
        desiredRotatorPosition = newPosition;
        rotator.setTargetPosition(newPosition);
    }

    public void adjustHoodBasedOnDistance(double dist) {
        if (hood != null) {
            if (dist < FIRST_DISTANCE_THRESHOLD) {
                hood.setPosition(CLOSE_HOOD_POSITION);
            } else if (dist < SECOND_DISTANCE_THRESHOLD) {
                hood.setPosition(MID_HOOD_POSITION);
            } else {
                hood.setPosition(FAR_HOOD_POSITION);
            }
        }
    }

    /**
     * Updates limelight-based rotator adjustment during shooting states
     * Call this continuously in shooting states for auto-adjustment
     * Uses last valid values as fallback when limelight doesn't see target
     */
    public void updateLimelightAdjustments() {
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            if (ll != null && ll.isValid()) {
                // Limelight sees target - use current values
                txDeg = ll.getTx();
                tyDeg = ll.getTy();

                // Store valid values for fallback
                lastValidTx = txDeg;
                lastValidTy = tyDeg;

                // Calculate distance for rotator offset calculation
                double currentDistance = getDist(tyDeg);
                if (currentDistance > 0) {
                    lastValidDistance = currentDistance;
                    hasValidLimelightData = true;

                    // Adjust rotator based on horizontal offset and distance (for offset calculation)
                    adjustRotator(txDeg, currentDistance);
                }
            } else {
                // Limelight doesn't see target - use last valid values if available
                if (hasValidLimelightData) {
                    // Use last known good values (from previous successful detection)
                    adjustRotator(lastValidTx, lastValidDistance);
                }
                // If no valid data ever, do nothing (keep current rotator position)
            }
        }
    }

    /**
     * Resets limelight data (useful when starting a new shooting sequence)
     */
    public void resetLimelightData() {
        hasValidLimelightData = false;
        lastValidTx = 0.0;
        lastValidTy = 0.0;
        lastValidDistance = 0.0;
    }

}