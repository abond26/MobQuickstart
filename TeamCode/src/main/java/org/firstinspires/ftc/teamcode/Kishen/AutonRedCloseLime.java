package org.firstinspires.ftc.teamcode.Kishen;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Autonomous(name = "limelight red", group = "Limelight")
public class AutonRedCloseLime extends OpMode {
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

    private Servo hood;
    private int limeHeight = 33;
    private int offset = 28;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 0.6; // tune this

    // Hood adjustment constants (from TesterinoRed - exact match)
    private static final double FIRST_DISTANCE_THRESHOLD = 140.0;
    private static final double SECOND_DISTANCE_THRESHOLD = 200;
    private static final double CLOSE_HOOD_POSITION = 0.0339; // Hood position for close shots
    private static final double MID_HOOD_POSITION = 0.203 + 0.0167;
    private static final double FAR_HOOD_POSITION = 0.25 + 0.0129; // Hood position for far shots

    private int y = tagHeight - limeHeight;
    //Rotator var
    int motor180Range = 910;
    int limelightUpAngle = 25;
    private static final int DEGREES_270_TICKS = 1365; // 270 degrees in ticks (910 * 1.5)
    private static final int MAX_ROTATOR_POSITION = DEGREES_270_TICKS; // Physical limit
    private static final int MIN_ROTATOR_POSITION = -DEGREES_270_TICKS; // Physical limit
    private int vMultiplier = 9;
    private Limelight3A limelight;

    // Store last valid limelight values for fallback
    private double lastValidTx = 0.0;
    private double lastValidTy = 0.0;
    private double lastValidDistance = 0.0;
    private boolean hasValidLimelightData = false;

    // Rotator adjustment safeguards to prevent 180-degree wrong-direction rotations
    private static final double TX_DEADBAND = 1.0; // Don't adjust if tx is less than 1 degree
    private static final int MIN_MOVEMENT_THRESHOLD = 10; // Don't adjust if movement is less than 10 ticks
    private static final int MAX_ADJUSTMENT_PER_CALL = 100; // Clamp adjustment to max 100 ticks per call
    private static final int ROTATOR_SETTLE_THRESHOLD = 20; // Only adjust when within 20 ticks of target
    private int desiredRotatorPosition = 0; // Track desired position separately to prevent cumulative errors
    private int adjustmentLoopCounter = 0;
    private static final int ADJUSTMENT_RATE_LIMIT = 3; // Only adjust every 3 loops (rate limiting)
    private com.qualcomm.robotcore.util.ElapsedTime lastAdjustmentTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    // Rotator physical limits to prevent 360-degree rotations and damage

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

    }

    PathState pathState;
    private final Pose startPose = new Pose(117.6, 130, Math.toRadians(36.5));
    private final Pose shootPose1 = new Pose(85, 88, Math.toRadians(46));
    private final Pose collect1thingstart=new Pose(85, 84, Math.toRadians(0));


    private final Pose collect1thing = new Pose(120, 84, Math.toRadians(0));
    private final Pose openGateStart = new Pose(120, 77, Math.toRadians(90));
    private final Pose openGateEnd = new Pose(122, 77, Math.toRadians(90));
    private final Pose shootPose2 = new Pose( 87, 84, Math.toRadians(46));


    private final Pose collect2Start = new Pose(88, 57.5, Math.toRadians(0));
    private final Pose collect2End = new Pose(127, 57.5, Math.toRadians(0));
    private final Pose shootBall3 = new Pose(87, 84, Math.toRadians(46));
    private final Pose collect3start=new Pose(95, 35, Math.toRadians(0));


    private final Pose collect3end = new Pose(127, 35, Math.toRadians(0));
    private final Pose shootBall4 = new Pose(87, 84, Math.toRadians(46));
    private final Pose park = new Pose(103, 84, Math.toRadians(46));



    private PathChain shoot1, goToCollect1, collect1, shoot2, goToCollect2, collect2, shoot3, awayfromGate, goToCollect3, collect3, shoot4, goToGate, openGate, goToCollect4, collect4, shoot5, parking;

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
//        awayfromGate = follower.pathBuilder()
//                .addPath(new BezierLine(collect1thing, awayFromGate))
//                .setLinearHeadingInterpolation(collect1thing.getHeading(), awayFromGate.getHeading())
//                .build();

        goToGate = follower.pathBuilder()
                .addPath(new BezierLine(collect1thing, openGateStart))
                .setLinearHeadingInterpolation(collect1thing.getHeading(), openGateStart.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(openGateStart, openGateEnd))
                .setLinearHeadingInterpolation(openGateStart.getHeading(), openGateEnd.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(openGateEnd, shootPose2))
                .setLinearHeadingInterpolation(openGateEnd.getHeading(), shootPose2.getHeading())
                .build();


        goToCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, collect2Start))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), collect2Start.getHeading())
                .build();

        collect2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2Start, collect2End))
                .setLinearHeadingInterpolation(collect2Start.getHeading(), collect2End.getHeading())
                .build();



        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(collect2End, shootBall3))
                .setLinearHeadingInterpolation(collect2End.getHeading(), shootBall3.getHeading())
                .build();
        goToCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(shootBall3, collect3start))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), collect3start.getHeading())
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3start, collect3end))
                .setLinearHeadingInterpolation(collect3start.getHeading(), collect3end.getHeading())
                .build();


        shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(collect3end, shootBall4))
                .setLinearHeadingInterpolation(collect3end.getHeading(), shootBall4.getHeading())
                .build();

        parking=follower.pathBuilder()
                .addPath(new BezierLine(shootBall4, park))
                .setLinearHeadingInterpolation(shootBall4.getHeading(), park.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                // Try to use limelight for initial adjustment, fallback to hardcoded values
                launcher.setVelocity(2075);
                hood.setPosition(0.275);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(shoot1);
                setPathState(AutonRedCloseLime.PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                // Sync desired position with current position when entering shooting state
                if (!shoot2Started) {
                    desiredRotatorPosition = rotator.getCurrentPosition();
                }
                // Continuously adjust based on limelight during shooting
                updateLimelightAdjustments();
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2){
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(-1);
                    if (pathTimer.getElapsedTimeSeconds()>3.5) {
                        setPathState(AutonRedCloseLime.PathState.collection);
                    }
                }
                break;
            case gotocollect:
                if(!follower.isBusy())
                {
                    follower.followPath(goToCollect1);
                    setPathState(AutonRedCloseLime.PathState.collection);
                }
                break;


            case collection:

                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    launcher.setVelocity(1775);
                    hood.setPosition(0.300);
                    theWheelOfTheOx.setPower(1);
                    tree.setPower(1);
                    follower.followPath(collect1);
                    setPathState((AutonRedCloseLime.PathState.goTowardsGate));
                }
                break;
//            case goAwayFromGate:
//                if (!follower.isBusy() && !goAwayFromGateStarted) {
//                    follower.followPath(awayfromGate);
//                    goAwayFromGateStarted = true; // Mark as started to prevent calling again
//                }
//                if (!follower.isBusy() && goAwayFromGateStarted && pathTimer.getElapsedTimeSeconds() > 5) {
//                    setPathState(PathState.goTowardsGate);
//                }
//                break;
            case goTowardsGate:
                if (!follower.isBusy() && !goTowardsGateStarted) {
                    follower.followPath(goToGate);
                    goTowardsGateStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && goTowardsGateStarted && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(AutonRedCloseLime.PathState.opengate);
                }
                break;
            case opengate:
                if (!follower.isBusy() && !opengateStarted) {
                    follower.followPath(openGate);
                    opengateStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && opengateStarted && pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(AutonRedCloseLime.PathState.shoot);
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
                    launcher.setVelocity(1775);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot2Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot2Started) {
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>4.5) {
                        setPathState((AutonRedCloseLime.PathState.collectAgain));
                    }
                }
                break;

            case collectAgain:
                if (!follower.isBusy()) {
                    follower.followPath(goToCollect2);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    setPathState((AutonRedCloseLime.PathState.collectAgainEnd));
                }
                break;
            case collectAgainEnd:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    launcher.setVelocity(1750);
                    follower.followPath(collect2);
                    tree.setPower(1);
                    hood.setPosition(0.315);
                    theWheelOfTheOx.setPower(1);
                    //theWheelOfTheOx.setPower(0.005);
                    //hood.setPosition(0.225);
                    setPathState((AutonRedCloseLime.PathState.shootAgain));
                }
                break;
            case shootAgain:
                // Sync desired position with current position when entering shooting state
                if (!shoot3Started) {
                    desiredRotatorPosition = rotator.getCurrentPosition();
                }
                // Continuously adjust based on limelight during shooting
                updateLimelightAdjustments();
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5 && !shoot3Started) {
                    follower.followPath(shoot3);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot3Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot3Started) {
                    if(pathTimer.getElapsedTimeSeconds()>4) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>5)
                    {
                        setPathState((AutonRedCloseLime.PathState.collectAgainAgain));
                    }
                }
                break;
            case collectAgainAgain:
                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(goToCollect3);
                    setPathState((AutonRedCloseLime.PathState.collectAgainAgainEnd));
                }
                break;
            case collectAgainAgainEnd:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    launcher.setVelocity(1775);
                    follower.followPath(collect3);
                    tree.setPower(1);
                    hood.setPosition(0.315);
                    theWheelOfTheOx.setPower(1);
                    //theWheelOfTheOx.setPower(0.005);
                    //hood.setPosition(0.225);
                    setPathState((AutonRedCloseLime.PathState.shootAgainAgain));
                }
                break;
            case shootAgainAgain:
                // Sync desired position with current position when entering shooting state
                if (!shoot4Started) {
                    desiredRotatorPosition = rotator.getCurrentPosition();
                }
                // Continuously adjust based on limelight during shooting
                updateLimelightAdjustments();
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5 && !shoot4Started) {
                    follower.followPath(shoot4);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot4Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot4Started) {
                    if(pathTimer.getElapsedTimeSeconds()>4) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>5)
                    {
                        setPathState((AutonRedCloseLime.PathState.parklol));
                    }
                }
                break;
            case parklol:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1 && !parkingStarted) {
                    follower.followPath(parking);
                    parkingStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && parkingStarted && pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState((AutonRedCloseLime.PathState.done));
                }
                break;
            case done:
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
    }

    @Override
    public void init() {
        pathState = PathState.start;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
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
        hood.scaleRange(0,0.0328);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorStartPosition=0;
        rotator.setTargetPosition(rotatorStartPosition);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);
        desiredRotatorPosition = rotatorStartPosition; // Initialize desired position

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize Limelight (pipeline 0 for red side)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(0);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("jolly crusader velocity", launcher.getVelocity());
        telemetry.addData("Rotator Target", rotator.getTargetPosition());
        telemetry.addData("Rotator Current", rotator.getCurrentPosition());
        // Limelight telemetry
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            if (ll != null && ll.isValid()) {
                telemetry.addLine("Limelight: DETECTING");
                telemetry.addData("LL tx", String.format("%.2f", ll.getTx()));
                telemetry.addData("LL ty", String.format("%.2f", ll.getTy()));
                telemetry.addData("LL Distance", String.format("%.1f", getDist(ll.getTy())));
            } else {
                telemetry.addLine("Limelight: NOT DETECTING");
                if (hasValidLimelightData) {
                    telemetry.addData("Using Fallback", "YES");
                } else {
                    telemetry.addData("Using Fallback", "NO - No valid data");
                }
            }
        }
    }

    public double getDist(double tyDeg) {
        // Use corrected distance formula from TesterinoBlue
        double tyRad = Math.abs(Math.toRadians(tyDeg + limelightUpAngle));
        double dist = y / Math.tan(tyRad);
        double realDist = 0.55 * dist + 40.3; // Correction formula from TesterinoBlue
        return realDist;
    }
    public double calcVelocity(double dist) {
        // Use piecewise linear formula from TesterinoRed for accurate velocity calculation
        double velocity;
        if (dist < FIRST_DISTANCE_THRESHOLD) {
            velocity = (4.94 * dist + 1008)*0.925;
        } else if (dist < SECOND_DISTANCE_THRESHOLD) {
            velocity = (4.22 * dist + 1129)*0.925;
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
    public void adjustRotator(double tx, double distance) {
        // CRITICAL SAFEGUARDS: Prevent 180-degree wrong-direction rotations
        // These are necessary in autonomous (TesterinoRed works in teleop where human can intervene)
        
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
        // This prevents offset accumulation that causes drift to the right
        desiredRotatorPosition = currentPos;
        
        // 5. Match TesterinoRed.java calculation exactly
        double fracOfFullCircum = Math.toRadians(tx) / (Math.PI);
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
        
        // 8. Distance-based offset (adjusted to compensate for rightward bias)
        // Reduced from 14/15 to correct for shooting slightly to the right
        int offset = 10;
        if (distance > 200) {
            offset = 11;
        }
        
        // 9. Calculate from CURRENT position (like TesterinoRed) to prevent offset accumulation
        // This prevents the rotator from drifting too far right after shooting
        int newPosition = currentPos + adjustment + offset;
        
        // 9. CRITICAL: Clamp final position to physical limits to prevent 360-degree rotations
        // This ensures the rotator never goes beyond ±270 degrees
        if (newPosition > MAX_ROTATOR_POSITION) {
            newPosition = MAX_ROTATOR_POSITION;
        }
        if (newPosition < MIN_ROTATOR_POSITION) {
            newPosition = MIN_ROTATOR_POSITION;
        }
        
        // 10. Update desired position and set target
        desiredRotatorPosition = newPosition;
        rotator.setTargetPosition(newPosition);
    }

    public void adjustHoodBasedOnDistance(double dist) {
        // Use three-tier hood adjustment from TesterinoRed for precise shooting
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
     * Updates limelight-based adjustments (rotator, velocity, hood) during shooting states
     * Call this continuously in shooting states for auto-adjustment
     * Uses last valid values as fallback when limelight doesn't see target
     * Matches TesterinoRed.java approach - simple and direct
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

                // Calculate distance and store it
                double currentDistance = getDist(tyDeg);
                if (currentDistance > 0) {
                    lastValidDistance = currentDistance;
                    hasValidLimelightData = true;

                    // CRITICAL: Only adjust rotator when we have valid detection
                    // This prevents drift when Limelight loses sight
                    adjustRotator(txDeg, currentDistance);

                    // Update velocity and hood based on distance
                    launcher.setVelocity(calcVelocity(currentDistance));
                    adjustHoodBasedOnDistance(currentDistance);
                }
            } else {
                // Limelight doesn't see target
                // CRITICAL FIX: Don't adjust rotator with fallback values - this causes drift
                // Only use fallback for velocity and hood (less critical for alignment)
                if (hasValidLimelightData && lastValidDistance > 0) {
                    // Use last known good values for velocity and hood only
                    launcher.setVelocity(calcVelocity(lastValidDistance));
                    adjustHoodBasedOnDistance(lastValidDistance);
                    // Do NOT adjust rotator - keep it at current position to prevent drift
                }
                // If no valid data ever, do nothing (keep current settings from state initialization)
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