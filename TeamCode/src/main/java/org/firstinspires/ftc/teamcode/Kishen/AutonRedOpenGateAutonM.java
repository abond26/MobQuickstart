package org.firstinspires.ftc.teamcode.Kishen;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
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
import org.firstinspires.ftc.teamcode.pedroPathing.WORKPLS;

@Autonomous(name = "Gate movement", group = "zzzzz")
public class AutonRedOpenGateAutonM extends OpMode {
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;

    // Flags to prevent path oscillation
    private boolean shoot2Started = false;
    private boolean shoot3Started = false;
    private boolean shoot4Started = false;
    private boolean shoot5Started = false;
    private boolean manualRotatorAdjustment = false; // Track if manual adjustment was just made
    private int gateCycleCount = 0; // Track how many times we've done gate open → shoot cycle

    // Limelight adjustment improvements
    private int desiredRotatorPosition = 0; // Track desired position to fix cumulative adjustment
    private int adjustmentLoopCounter = 0; // Rate limiting counter
    private static final int ADJUSTMENT_RATE_LIMIT = 8; // Adjust every 8 loops (~4 times/second)
    private static final int MIN_MOVEMENT_THRESHOLD = 5; // Minimum ticks to move
    private static final double TX_DEADBAND = 0.5; // Deadband in degrees
    private static final int TX_SMOOTHING_SAMPLES = 3; // Number of samples for smoothing
    private double[] txHistory = new double[TX_SMOOTHING_SAMPLES];
    private boolean[] txHistoryValid = new boolean[TX_SMOOTHING_SAMPLES]; // Track which samples are valid
    private int txHistoryIndex = 0;
    private int txHistoryCount = 0; // Count of valid samples in history
    private Timer lastAdjustmentTimer = new Timer(); // Track time since last adjustment
    private static final boolean VERBOSE_DEBUG = false; // Set to true for detailed debug telemetry

    private Servo hood;
    // Limelight constants (matching TesterinoBlue exactly)
    private int limeHeight = 35;
    private int offset = 28;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 0.6; // tune this

    // Hood adjustment constants (from TesterinoBlue)
    private static final double DISTANCE_THRESHOLD = 180.0; // Change hood when distance > 180 inches
    private static final double CLOSE_HOOD_POSITION = 0.4404; // Hood position for close shots (matches TesterinoBlue)
    private static final double FAR_HOOD_POSITION = 0.5; // Hood position for far shots

    private int y = tagHeight - limeHeight;
    //Rotator var (matching TesterinoBlue exactly)
    int motor180Range = 630;  // Changed from 910 to match TesterinoBlue
    int limelightUpAngle = 20;  // Changed from 25 to match TesterinoBlue
    private int vMultiplier = 9;
    private Limelight3A limelight;

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
        collection,
        shoot,
        collectAgain,
        collectAgainEnd,
        opengatestart,
        opengateend,


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
    private final Pose startPose = new Pose(119.6, 126.7, Math.toRadians(37));
    private final Pose shootPose1 = new Pose(92, 82, Math.toRadians(0));


    private final Pose collect1thing = new Pose(128, 82, Math.toRadians(0));
    private final Pose shootPose2 = new Pose( 92, 84, Math.toRadians(0));  // "shoot" state - if too far left, increase X (move right)


    private final Pose collect2Start = new Pose(92, 55, Math.toRadians(0));
    private final Pose collect2End = new Pose(136, 55, Math.toRadians(0));
    private final Pose shootPose3 = new Pose( 92, 84, Math.toRadians(0));
    private final Pose openGateControlPoint = new Pose(117.818367347, 60.4122448979592, Math.toRadians(180));
    private final Pose openGateStart = new Pose(113, 60, Math.toRadians(37));
    private final Pose openGateEnd = new Pose(132, 60, Math.toRadians(37));
    private final Pose shootBall3 = new Pose(87, 28, Math.toRadians(0));  // Shooting position after gate

    private PathChain shoot1, collect1, shoot2, goToCollect2, collect2, goToGate, openGate, shoot3, goBackToGate;

    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();


        collect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, collect1thing))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), collect1thing.getHeading())
                .build();



        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1thing, shootPose2))
                .setLinearHeadingInterpolation(collect1thing.getHeading(), shootPose2.getHeading())
                .build();


        goToCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, collect2Start))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), collect2Start.getHeading())
                .build();

        collect2 = follower.pathBuilder()
                .addPath(new BezierLine(collect2Start, collect2End))
                .setLinearHeadingInterpolation(collect2Start.getHeading(), collect2End.getHeading())
                .build();

        goToGate = follower.pathBuilder()
                .addPath(new BezierCurve(collect2End, openGateControlPoint, openGateStart))
                .setLinearHeadingInterpolation(collect2End.getHeading(), openGateStart.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(openGateStart, openGateEnd))
                .setLinearHeadingInterpolation(openGateStart.getHeading(), openGateEnd.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(openGateEnd, shootBall3))
                .setLinearHeadingInterpolation(openGateEnd.getHeading(), shootBall3.getHeading())
                .build();

        goBackToGate = follower.pathBuilder()
                .addPath(new BezierLine(shootBall3, openGateStart))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), openGateStart.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                // Set initial velocity to 1700 (will be updated by limelight if it sees target)
                launcher.setVelocity(1700);
                // Use hardcoded initial values - limelight will adjust once we're in position
                adjustRotator(-24.5);
                hood.setPosition(0.175);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(shoot1);
                setPathState(AutonRedOpenGateAutonM.PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                // Set velocity to 1700 (will be updated by limelight if it sees target)
                launcher.setVelocity(1700);
                // Continuously adjust based on limelight during shooting
                updateLimelightAdjustments();
                if (!follower.isBusy()){
                    // Only start shooting when aligned (limelight sees target and tx is close to 0)
                    // OR if timeout (4 seconds) - don't wait forever
                    boolean isAligned = false;
                    if (limelight != null) {
                        LLResult ll = limelight.getLatestResult();
                        if (ll != null && ll.isValid()) {
                            double currentTx = ll.getTx();
                            // Consider aligned if tx is within 1.5 degrees of center
                            if (Math.abs(currentTx) < 1.5) {
                                isAligned = true;
                            }
                        }
                    }

                    // Shoot if aligned OR if timeout (4 seconds) - ensure velocity and motors are set
                    if (isAligned || pathTimer.getElapsedTimeSeconds() > 4.0) {
                        launcher.setVelocity(1700); // Ensure velocity is set
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                        setPathState(AutonRedOpenGateAutonM.PathState.collection);
                    }
                    // If not aligned and not timeout, stay in this state and keep adjusting
                }
                break;

            case collection:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    adjustRotator(-5);  // Changed: shoot is too far left, so move rotator right (more positive/less negative)
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    theWheelOfTheOx.setPower(1);
                    tree.setPower(1);
                    follower.followPath(collect1);
                    setPathState((AutonRedOpenGateAutonM.PathState.shoot));
                }
                break;
            case shoot:
                // Set velocity to 1700 initially (will be updated by limelight if it sees target)
                launcher.setVelocity(1700);
                // Add manual adjustment before limelight takes over (helps initial alignment)
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    adjustRotator(-5);  // Changed: shoot is too far left, so move rotator right (more positive/less negative)
                }
                // Continuously adjust based on limelight during shooting
                updateLimelightAdjustments();

                // Only follow path once - use flag to prevent oscillation
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && !follower.isBusy() && !shoot2Started) {
                    follower.followPath(shoot2);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot2Started = true; // Mark as started
                }

                // Only start shooting (theWheelOfTheOx) when aligned and path is done
                // OR if timeout (4 seconds) - shoot anyway
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    boolean isAligned = false;
                    if (limelight != null) {
                        LLResult ll = limelight.getLatestResult();
                        if (ll != null && ll.isValid()) {
                            double currentTx = ll.getTx();
                            // Consider aligned if tx is within 1.5 degrees of center
                            if (Math.abs(currentTx) < 1.5) {
                                isAligned = true;
                            }
                        }
                    }

                    // Shoot if aligned OR if timeout (4 seconds) - ensure velocity and motors are set
                    if (isAligned || pathTimer.getElapsedTimeSeconds() > 4.0) {
                        launcher.setVelocity(1700); // Ensure velocity is set
                        tree.setPower(1); // Ensure tree is running
                        theWheelOfTheOx.setPower(-1);
                        setPathState((AutonRedOpenGateAutonM.PathState.collectAgain));
                    }
                }
                break;
            case collectAgain:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(goToCollect2);
                    setPathState((AutonRedOpenGateAutonM.PathState.collectAgainEnd));
                }
                break;
            case collectAgainEnd:
                if (!follower.isBusy()) {
                    // Set initial velocity to 3500 (will be updated by limelight if it sees target)
                    launcher.setVelocity(3500);
                    adjustRotator(-45);  // Changed: shootAgain is too far right, so move rotator left (more negative)
                    follower.followPath(collect2);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(1);
                    //theWheelOfTheOx.setPower(0.005);
                    setPathState((AutonRedOpenGateAutonM.PathState.shootAgain));
                }
                break;
            case opengatestart:
                // Turn intake on while going to gate
                tree.setPower(1);
                theWheelOfTheOx.setPower(1);
                // Check if we're coming from shooting position (need to go back to gate) or from collection
                if (!follower.isBusy()) {
                    // If we're at shooting position, use goBackToGate, otherwise use goToGate
                    double currentX = follower.getPose().getX();
                    double currentY = follower.getPose().getY();
                    // Check if we're near the shooting position (shootBall3 is at 87, 28)
                    if (Math.abs(currentX - 87) < 10 && Math.abs(currentY - 28) < 10) {
                        // Coming from shooting position, go back to gate
                        follower.followPath(goBackToGate);
                    } else {
                        // Coming from collection, go to gate normally
                        follower.followPath(goToGate);
                    }
                    setPathState(AutonRedOpenGateAutonM.PathState.opengateend);
                }
                break;
            case opengateend:
                // Keep intake on while opening gate
                tree.setPower(1);
                theWheelOfTheOx.setPower(1);
                if (!follower.isBusy())
                {
                    follower.followPath(openGate);
                    // Keep gate open for 2.5 seconds
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        gateCycleCount++; // Increment cycle count
                        setPathState(AutonRedOpenGateAutonM.PathState.shootAgain);
                    }
                }
                break;

            case shootAgain:
                // Set velocity to 3500 initially (will be updated by limelight if it sees target)
                launcher.setVelocity(3500);
                // Add manual adjustment before limelight takes over (helps initial alignment)
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    adjustRotator(-45);  // Changed: shootAgain is too far right, so move rotator left (more negative)
                }
                // Continuously adjust based on limelight during shooting - THIS IS CRITICAL FOR ACCURACY
                updateLimelightAdjustments();
                if (pathTimer.getElapsedTimeSeconds() > 1.5 && !follower.isBusy() && !shoot3Started) {
                    // Only follow path once - use flag to prevent oscillation
                    follower.followPath(shoot3);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot3Started = true; // Mark as started
                }

                // Only start shooting (theWheelOfTheOx) when aligned and path is done
                // Add timeout so it doesn't wait forever
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    boolean isAligned = false;
                    if (limelight != null) {
                        LLResult ll = limelight.getLatestResult();
                        if (ll != null && ll.isValid()) {
                            double currentTx = ll.getTx();
                            // Consider aligned if tx is within 1.5 degrees of center
                            if (Math.abs(currentTx) < 1.5) {
                                isAligned = true;
                            }
                        }
                    }

                    // Shoot if aligned OR if timeout (4 seconds) - ensure velocity and motors are set
                    if (isAligned || pathTimer.getElapsedTimeSeconds() > 4.0) {
                        launcher.setVelocity(3500); // Ensure velocity is set (limelight will update if it sees target)
                        tree.setPower(1); // Ensure tree is running
                        theWheelOfTheOx.setPower(-1);
                        // After shooting, wait a bit then go to gate (if cycles remaining) or finish
                        if (pathTimer.getElapsedTimeSeconds() > 5.5) {  // Give time for shooting (1.5 seconds)
                            if (gateCycleCount < 2) {
                                // Go to gate to open it, then cycle back to shoot
                                setPathState((AutonRedOpenGateAutonM.PathState.opengatestart));
                            } else {
                                // Done with 2 gate cycles, autonomous complete
                                setPathState((AutonRedOpenGateAutonM.PathState.done));
                            }
                        }
                    }
                }
                break;
            // Removed states: collectAgainAgain, collectAgainAgainEnd, shootAgainAgain, 
            // collectAgainAgainAgain, collectAgainAgainAgainEnd, shootAgainAgainAgain, parklol
            // These paths were deleted, so autonomous ends after gate cycles complete

            case done:
                break;

        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        // Reset flags when state changes
        shoot2Started = false;
        shoot3Started = false;
        shoot4Started = false;
        shoot5Started = false;
        manualRotatorAdjustment = false; // Reset manual adjustment flag
        adjustmentLoopCounter = 0; // Reset rate limiting counter
        txHistoryIndex = 0; // Reset smoothing history index
        txHistoryCount = 0; // Reset valid sample count
        // Clear smoothing history when state changes
        for (int i = 0; i < TX_SMOOTHING_SAMPLES; i++) {
            txHistoryValid[i] = false;
        }
        // Reset desired position to current position when state changes
        if (rotator != null) {
            desiredRotatorPosition = rotator.getCurrentPosition();
        }
        // Reset gate cycle count when starting a new gate sequence
        if (newState == PathState.collectAgainEnd) {
            gateCycleCount = 0;
        }
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
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);  // Match TesterinoBlue
        rotator.setPower(1.0);  // Ensure power is set
        desiredRotatorPosition = 0; // Initialize desired position
        lastAdjustmentTimer.resetTimer(); // Reset timer

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);  // Changed from 0 to 1 to match blue side (verify this is correct for your setup)
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
        // Essential telemetry - keep it clean
        telemetry.addData("State", pathState.toString());
        telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)",
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("Launcher", (int)launcher.getVelocity());
        telemetry.addData("Rotator", rotator.getCurrentPosition() + "/" + rotator.getTargetPosition());
        telemetry.addData("Gate Cycles", gateCycleCount + "/2");  // Show cycle progress

        // Verbose debug telemetry (only if enabled)
        if (VERBOSE_DEBUG) {
            telemetry.addData("Path Time", String.format("%.2f", pathTimer.getElapsedTimeSeconds()));
            telemetry.addData("TX", String.format("%.2f", txDeg));
            telemetry.addData("TY", String.format("%.2f", tyDeg));
            if (Math.abs(txDeg) > 0.1) {
                // Match actual calculation (full circle)
                double fracOfFullCircum = Math.toRadians(txDeg) / (2 * Math.PI);
                int adjustment = (int) (fracOfFullCircum * motor180Range * 2);
                telemetry.addData("Calc Adj", adjustment);
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
        // Use simpler linear formula from TesterinoBlue
        double velocity = 3.30933 * dist + 1507.01002;
        return velocity;
    }

    public void intake(double intakePower){
        tree.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-0.3);
        }
    }
    public void adjustRotator(double tx) {
        // Match TesterinoBlue exactly - simple and direct
        // Only skip if already very close to 0 (deadband) - but allow manual adjustments
        // Manual adjustments (like 25.5, 15, 9) should always work, only skip limelight adjustments when close
        boolean isManualAdjustment = Math.abs(tx) > 10; // If tx > 10, it's likely a manual adjustment

        // Apply deadband (increased to 0.5 degrees)
        if (!isManualAdjustment && Math.abs(tx) < TX_DEADBAND) {
            return; // Already aligned, don't adjust (only for limelight)
        }

        // Calculate adjustment - match TesterinoBlue exactly (full circle)
        double fracOfFullCircum = Math.toRadians(tx) / (2 * Math.PI);
        int adjustment = (int) (fracOfFullCircum * motor180Range * 2);

        // Apply minimum movement threshold - don't adjust if movement is too small
        if (!isManualAdjustment && Math.abs(adjustment) < MIN_MOVEMENT_THRESHOLD) {
            return; // Movement too small, skip adjustment
        }

        // Fix cumulative adjustment: calculate from desired position, not current position
        // This prevents "chasing" when rotator hasn't reached previous target
        int currentPos = rotator.getCurrentPosition();
        int currentTarget = rotator.getTargetPosition();

        // If rotator is close to its current target (within 10 ticks), use current position
        // Otherwise, use the desired position to prevent cumulative errors
        int basePosition;
        if (Math.abs(currentPos - currentTarget) < 10) {
            // Rotator is close to target, use current position
            basePosition = currentPos;
            desiredRotatorPosition = currentPos; // Update desired to match
        } else {
            // Rotator is still moving, use desired position to prevent cumulative adjustment
            basePosition = desiredRotatorPosition;
        }

        int newPosition = basePosition + adjustment - offset;
        desiredRotatorPosition = newPosition; // Update desired position

        // Always set target - let it continuously adjust to drive tx toward 0
        // Make sure rotator is in correct mode and has power
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setTargetPosition(newPosition);
        rotator.setPower(1.0);

        // Mark if this is a manual adjustment
        if (isManualAdjustment) {
            manualRotatorAdjustment = true;
            // For manual adjustments, reset desired position to current
            desiredRotatorPosition = currentPos;
        }

        // Essential telemetry only
        telemetry.addData("Rotator Pos", currentPos);
        telemetry.addData("Rotator Target", newPosition);
        telemetry.addData("Rotator TX", String.format("%.2f", tx));

        // Verbose debug telemetry (only if enabled)
        if (VERBOSE_DEBUG) {
            int actualTarget = rotator.getTargetPosition();
            telemetry.addData("Adj", adjustment);
            telemetry.addData("Desired Pos", desiredRotatorPosition);
            telemetry.addData("Target Match", newPosition == actualTarget);
            telemetry.addData("At Target", Math.abs(currentPos - actualTarget) < 3);
            telemetry.addData("Manual Adj", isManualAdjustment);
        }
    }

    public void adjustHoodBasedOnDistance(double distance) {
        if (hood != null) {
            if (distance > DISTANCE_THRESHOLD) {
                hood.setPosition(FAR_HOOD_POSITION);
                if (VERBOSE_DEBUG) {
                    telemetry.addData("Hood", "FAR");
                }
            } else {
                hood.setPosition(CLOSE_HOOD_POSITION);
                if (VERBOSE_DEBUG) {
                    telemetry.addData("Hood", "CLOSE");
                }
            }
            if (VERBOSE_DEBUG) {
                telemetry.addData("Hood Dist", String.format("%.1f", distance));
            }
        }
    }

    /**
     * Updates limelight-based adjustments (rotator, velocity, hood) during shooting states
     * Call this continuously in shooting states for auto-adjustment
     * Only updates when limelight actually sees target (keeps initial settings otherwise)
     */
    public void updateLimelightAdjustments() {
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            if (ll != null && ll.isValid()) {
                // Only show essential limelight data when valid
                telemetry.addData("LL TX", String.format("%.2f", ll.getTx()));
                telemetry.addData("LL TY", String.format("%.2f", ll.getTy()));
                telemetry.addData("LL Valid", "YES");  // Make it clear limelight sees target
                if (VERBOSE_DEBUG) {
                    telemetry.addData("LL TA", String.format("%.3f", ll.getTa()));
                }
            } else {
                telemetry.addData("LL Status", "No Target");  // Make it clear limelight doesn't see target
                telemetry.addData("LL Valid", "NO");
            }

            if (ll != null && ll.isValid()) {
                // Limelight sees target - use current values
                double rawTx = ll.getTx();
                tyDeg = ll.getTy();

                // Apply smoothing/filtering to tx to reduce noise
                txHistory[txHistoryIndex] = rawTx;
                if (!txHistoryValid[txHistoryIndex]) {
                    txHistoryCount++; // Increment count when adding new valid sample
                }
                txHistoryValid[txHistoryIndex] = true;
                txHistoryIndex = (txHistoryIndex + 1) % TX_SMOOTHING_SAMPLES;

                // Calculate smoothed tx (average of last N valid samples)
                double smoothedTx = 0.0;
                int validSamples = 0;
                for (int i = 0; i < TX_SMOOTHING_SAMPLES; i++) {
                    if (txHistoryValid[i]) { // Only count valid samples (0.0 is valid!)
                        smoothedTx += txHistory[i];
                        validSamples++;
                    }
                }
                if (validSamples > 0) {
                    smoothedTx = smoothedTx / validSamples;
                } else {
                    smoothedTx = rawTx; // Fallback to raw if no valid samples
                }

                txDeg = smoothedTx; // Use smoothed value

                // Store valid values for reference
                lastValidTx = txDeg;
                lastValidTy = tyDeg;

                // Only adjust rotator with limelight if:
                // 1. Manual adjustment was made and rotator has reached target (or timeout passed)
                // 2. OR no manual adjustment was made
                // This ensures manual adjustments complete before limelight fine-tunes
                boolean shouldUseLimelight = true;
                if (manualRotatorAdjustment) {
                    int currentPos = rotator.getCurrentPosition();
                    int targetPos = rotator.getTargetPosition();
                    // Improved timeout: 0.3 seconds (reduced from 0.5) and 10 ticks threshold (increased from 5)
                    if (Math.abs(currentPos - targetPos) < 10 || pathTimer.getElapsedTimeSeconds() > 0.3) {
                        manualRotatorAdjustment = false; // Manual adjustment complete, limelight can take over
                    } else {
                        shouldUseLimelight = false; // Still moving to manual position, don't override
                    }
                }

                // Rate limiting: only adjust every N loops or every 150ms
                adjustmentLoopCounter++;
                boolean shouldAdjustNow = false;
                if (shouldUseLimelight) {
                    if (adjustmentLoopCounter >= ADJUSTMENT_RATE_LIMIT ||
                            lastAdjustmentTimer.getElapsedTimeSeconds() > 0.15) {
                        shouldAdjustNow = true;
                        adjustmentLoopCounter = 0;
                        lastAdjustmentTimer.resetTimer();
                    }
                }

                // Only show verbose debug info
                if (VERBOSE_DEBUG) {
                    telemetry.addData("Use LL", shouldUseLimelight);
                    telemetry.addData("Adjust Now", shouldAdjustNow);
                    telemetry.addData("Raw TX", String.format("%.2f", rawTx));
                    telemetry.addData("Smoothed TX", String.format("%.2f", smoothedTx));
                }

                if (shouldUseLimelight && shouldAdjustNow) {
                    // Adjust rotator based on horizontal offset (only when we see target)
                    adjustRotator(txDeg);
                }

                // Calculate distance and update velocity/hood only when we see target
                double currentDistance = getDist(tyDeg);
                if (currentDistance > 0) {
                    lastValidDistance = currentDistance;
                    hasValidLimelightData = true;

                    // Update velocity and hood based on distance (only when limelight sees target)
                    launcher.setVelocity(calcVelocity(currentDistance));
                    adjustHoodBasedOnDistance(currentDistance);
                }
            }
            // If limelight doesn't see target, do nothing - keep current settings
            // (rotator stays where it is, velocity stays at initial 1700/3500)
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