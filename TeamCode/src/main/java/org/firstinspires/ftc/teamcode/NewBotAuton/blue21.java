package org.firstinspires.ftc.teamcode.NewBotAuton;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;
import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.LookUpTables.VelocityLookupTable;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.tests.ColorTesting;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "blue 21 ", group = "auto", preselectTeleOp = "BigBoyBlue")
public class blue21 extends OpMode {
    private static final int PIPELINENUM = 1;
    Pose target = new Pose(0, 144, Math.toRadians(144));
    Pose sillyTarget;
    private int rotatorStartPosition=0;
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;
    Robot robot;

    private Vision vision = null;
    private ColorTesting colorTesting = null;
    private Intake intakeSubsystem = null;
    private Turret turretSubsystem = null;
    private TransferGate gateSubsystem = null;
    private static final double BALL_DETECT_TIMEOUT_SEC = 3.0;

    // Flags to prevent path oscillation - ensure paths are only called once per state
    private boolean shoot1Started = false;
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

    private Servo hood, blocker;
    private int limeHeight = 33;
    private int offset = 28;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 1; // tune this
    RobotActions actions;

    // Hood adjustment constants (from TesterinoBlue)
    private static final double DISTANCE_THRESHOLD = 180.0;
    private static final double CLOSE_HOOD_POSITION = .2541; // Hood position for close shots
    private static final double FAR_HOOD_POSITION = 0.36; // Hood position for far shots

    private int y = tagHeight - limeHeight;
    //Rotator var
    int motor180Range = 910;
    int limelightUpAngle = 25;
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
        GateCollectionAgainAgain,
        shootAgainAgainAgainAgain,

        shoot3ToGate
    }

    PathState pathState;
    // Mirrored coordinates: blueX = 144 - redX, blueHeading = Math.PI - redHeading
    private final Pose startPose = new Pose(26.5, 134, Math.toRadians(233.5));
    private final Pose shootPose1 = new Pose(46, 97.5, Math.toRadians(241));
    //private final Pose collect1thingstart = new Pose(56, 59, Math.toRadians(180));
        private final Pose collect1thing = new Pose(20, 60, Math.toRadians(180));
    private final Pose goToCollect1ControlPoint = new Pose(52.26522653061225, 60.091326530612236, Math.toRadians(180));
    private final Pose shootPose2 = new Pose( 63, 85, Math.toRadians(131.5));

    // Control points for shoot2 path
    //private final Pose shoot2ControlPoint1 = new Pose(51, 76, Math.toRadians(180));
    private final Pose gateCollect1 = new Pose( 20, 69, Math.toRadians(160));
    //private final Pose inBetween1 = new Pose(44, 62, Math.toRadians(157.5));
    private final Pose shootPose2ToGateControlPoint = new Pose(37.27755102040817, 51.926530612244896);
    private final Pose shootBall3 = new Pose(63, 85, Math.toRadians(131.5));
    private final Pose inBetween2 = new Pose(44, 62, Math.toRadians(157.5));
    private final Pose gateCollect2 = new Pose( 15.5, 62, Math.toRadians(155));
    private final Pose shootBall4 = new Pose(60, 75, Math.toRadians(130));
    private final Pose gateCollect3 = new Pose( 15.5, 62, Math.toRadians(155));
    private final Pose shootBall5 = new Pose(49, 87, Math.toRadians(130));

    //private final Pose collect3start=new Pose(57, 86, Math.toRadians(180));
    //private final Pose shoot4ToCollect3ControlPoint = new Pose(41.254125340599455039340599455039, 82.36784741144412, Math.toRadians(180));

    //
    private final Pose collect3end = new Pose(24, 85, Math.toRadians(180));
    private final Pose collect3ControlPoint = new Pose(41.21632653061224, 85.83673469387755);
    private final Pose shootBall6 = new Pose(55, 120, Math.toRadians(147));

    private final Pose park = new Pose(41, 84, Math.toRadians(134));



    private PathChain shoot1, goToCollect1,shoot3ToGate,collect1, shoot2, GateCollect3, shoot6, InBetween1, InBetween2, GateCollect1, GateCollect2, shoot3, awayfromGate, goToCollect3, collect3, shoot4, goToGate, openGate, goToCollect4, collect4, shoot5, parking;

    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        collect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, goToCollect1ControlPoint, collect1thing))
                .setTangentHeadingInterpolation()
                .build();

//        collect1 = follower.pathBuilder()
//                .addPath(new BezierLine(collect1thingstart, collect1thing))
//                .setLinearHeadingInterpolation(collect1thingstart.getHeading(), collect1thing.getHeading())
//                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1thing, shootPose2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        GateCollect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, shootPose2ToGateControlPoint, gateCollect1))
                .setTangentHeadingInterpolation()
                .build();

//        GateCollect1 = follower.pathBuilder()
//                .addPath(new BezierLine(gateCollect1, gateCollect1))
//                .setLinearHeadingInterpolation(gateCollect1.getHeading(), gateCollect1.getHeading())
//                .build();

        shoot3ToGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall3, shootPose2ToGateControlPoint, gateCollect1))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect1, shootPose2ToGateControlPoint, shootBall3))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        GateCollect2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall3, shootPose2ToGateControlPoint, gateCollect2))
                .setTangentHeadingInterpolation()
                .build();
//        GateCollect2 = follower.pathBuilder()
//                .addPath(new BezierLine(gateCollect2, gateCollect2))
//                .setLinearHeadingInterpolation(gateCollect2.getHeading(), gateCollect2.getHeading())
//                .build();
        shoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect2, shootPose2ToGateControlPoint, shootBall4))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        GateCollect3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall4, shootPose2ToGateControlPoint, gateCollect3))
                .setTangentHeadingInterpolation()
                .build();
        shoot5 = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect3, shootPose2ToGateControlPoint, shootBall5))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall3, collect3ControlPoint, collect3end))
                .setTangentHeadingInterpolation()
                .build();

//        collect3 = follower.pathBuilder()
//                .addPath(new BezierLine(collect3start, collect3end))
//                .setLinearHeadingInterpolation(collect3start.getHeading(), collect3end.getHeading())
//                .build();
//
//
        shoot6 = follower.pathBuilder()
                .addPath(new BezierLine(collect3end, shootBall6))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
//
//        parking=follower.pathBuilder()
//                .addPath(new BezierLine(shootBall4, park))
//                .setLinearHeadingInterpolation(shootBall4.getHeading(), park.getHeading())
//                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                if (!shoot1Started) {
                    robot.gate.block();
                    robot.intake.powerON();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(shoot1);
                    robot.intake.up();
                    shoot1Started = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.gate.open();
                    setPathState(blue21.PathState.actuallyshoot1);
                }
                break;
            case actuallyshoot1:
                // Keep aiming during first-shot motion.
                robot.intake.shift();

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.5){
                        setPathState(PathState.collection);
                }
                break;


            case collection:

                if (!follower.isBusy() && !collectionStarted) {
                    robot.gate.block();
                    follower.followPath(collect1);
                    collectionStarted = true;
                }
                if ((arrived() || isNear(collect1thing, 3)) && collectionStarted) {
                    setPathState((blue21.PathState.shoot));
                }
                break;
            case shoot:
                if (!follower.isBusy() && !shoot2Started) {
                    robot.intake.up();
                    follower.followPath(shoot2);
                    shoot2Started = true;
                }
                if (shootinAuton() && shoot2Started) {
                    robot.gate.open();
                    robot.intake.powerON();
                    if(arrived() || isNear(shootPose2, 3)) {
                        robot.intake.shift();
                        setPathState((PathState.GateCollection));
                    }
                }
                break;
            case GateCollection:
                if (!follower.isBusy() && !gateCollectionStarted) {
                    robot.gate.block();
                    follower.followPath(GateCollect1);
                    gateCollectionStarted = true; // Mark as started to prevent calling again
                }

                Intake.DetectedColor color = robot.intake.AutonColor(telemetry);

                boolean full = (color != Intake.DetectedColor.UNKNOWN);
                boolean ready = full || pathTimer.getElapsedTimeSeconds() > 2.5;

                if ((arrived() || isNear(gateCollect1, 3)) && gateCollectionStarted && ready) {
                    setPathState((blue21.PathState.shootAgain));
                }
                break;
            case shootAgain:
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && !shoot3Started) {
                    follower.followPath(shoot3);
                    //tree.setPower(1);
                    shoot3Started = true; // Mark as started to prevent calling again
                }
                if (shootinAuton() && shoot3Started) {
                    robot.gate.open();
                    boolean close = arrived() || isNear(shootBall3, 3);
                    if(opModeTimer.getElapsedTimeSeconds()<24.5)
                    {
                        if(close) {
                            setPathState((blue21.PathState.GateCollection));
                        }
                    }
                    if(opModeTimer.getElapsedTimeSeconds()>24.5 && pathTimer.getElapsedTimeSeconds()>2.25)
                    {
                        robot.intake.shift();
                        if(close) {
                            setPathState(PathState.collectAgainAgainEnd);
                        }
                    }
                }
                break;
//TODO CONTINUE FROM HERE
            case GateCollectionAgain:
                if (!follower.isBusy() && !gateCollectionAgainStarted) {
                    robot.intake.shift();
                    follower.followPath(GateCollect2);
                    gateCollectionAgainStarted = true; // Mark as started to prevent calling again
                }
                if ((arrived() || isNear(gateCollect2, 3)) && gateCollectionAgainStarted) {
                    setPathState((blue21.PathState.shootAgainAgain));
                }
                break;
            case shootAgainAgain:
                if (!follower.isBusy() && !shoot4Started) {
                    follower.followPath(shoot4);
                    robot.intake.up();
                    shoot4Started = true;
                }
                if (shoot4Started && pathTimer.getElapsedTimeSeconds() > 2.75) {
                    setPathState((blue21.PathState.GateCollectionAgainAgain));
                }
                break;
            case GateCollectionAgainAgain:
                if (!follower.isBusy() && !gateCollectionAgainStarted) {
                    robot.intake.shift();
                    follower.followPath(GateCollect3);
                    gateCollectionAgainStarted = true;
                }
                if ((arrived() || isNear(gateCollect3, 3)) && gateCollectionAgainStarted) {
                    setPathState((blue21.PathState.shootAgainAgainAgain));
                }
                break;
            case shootAgainAgainAgain:
                if (!follower.isBusy() && !shoot4Started) {
                    follower.followPath(shoot5);
                    robot.intake.up();
                    shoot4Started = true;
                }
                if (shoot4Started && pathTimer.getElapsedTimeSeconds() > 2.75) {
                    setPathState((blue21.PathState.collectAgainAgainEnd));
                }
                break;

            case collectAgainAgainEnd:
                robot.gate.block();
                if (!follower.isBusy() && !collectionStarted) {
                    follower.followPath(collect3);
                    collectionStarted = true;
                }
                if ((arrived() || isNear(collect3end, 3)) && collectionStarted) {
                    setPathState((blue21.PathState.shootAgainAgainAgainAgain));
                }
                break;
            case shootAgainAgainAgainAgain:
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && !shoot5Started) {
                    follower.followPath(shoot6);
                    shoot5Started = true; // Mark as started to prevent calling again
                }
                if (shootinAuton() && shoot5Started) {
                    robot.gate.open();
                    if(pathTimer.getElapsedTimeSeconds()>2.95) {
                        setPathState((blue21.PathState.done));
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
        if (colorTesting != null &&
                (newState == PathState.GateCollection
                        || newState == PathState.GateCollectionAgain
                        || newState == PathState.GateCollectionAgainAgain)) {
            //colorTesting.resetBallCounter();
        }
        // Reset flags when state changes to allow paths to be called again in new state
        shoot1Started = false;
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
        follower.setStartingPose(startPose);
        buildPaths();

        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
        intakeSubsystem = robot.intake;
        turretSubsystem = robot.turret;
        gateSubsystem = robot.gate;
        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                robot.turret,
                robot.gate,
                robot.intake);
        robot.intake.up();
        robot.gate.block();
        sillyTarget = robot.chassisLocal.sillyTargetPose(target);

        telemetry.addLine("Good to go BLUE");
        telemetry.update();

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        Pose currentPose = follower.getPose();
        PoseStorage.savePose(currentPose);

        double dx = target.getX() - currentPose.getX();
        double dy = target.getY() - currentPose.getY();
        double dist = Math.sqrt(dx * dx + dy * dy);
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngle = angleToGoal - Math.toDegrees(currentPose.getHeading());
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;
        robot.turret.setRotatorToAngle(-turretAngle);

        robot.turret.setVelocity(VelocityLookupTable.getVelocity(dist));
        int zone = VelocityLookupTable.getZone(dist);
        if (zone == 1) robot.turret.setHoodPos(BlueUniversalConstants.CLOSE_HOOD_POSITION);
        else if (zone == 2) robot.turret.setHoodPos(BlueUniversalConstants.MID_HOOD_POSITION);
        else robot.turret.setHoodPos(BlueUniversalConstants.FAR_HOOD_POSITION);

        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", currentPose.getX());
        telemetry.addData("y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Dist to target", dist);
    }

    private boolean shootinAuton() {
        Pose pos = follower.getPose();
        double y = pos.getY();
        double x = pos.getX();
        double leftUpBound = -x + 144;
        double leftBottomBound = x - 48;
        double rightUpBound = x;
        double rightBottomBound = -x + 144 - 48;
        if (x < 72) {
            return (y > leftUpBound || y < leftBottomBound);
        } else {
            return (y > rightUpBound || y < rightBottomBound);
        }
    }

    private boolean isNear(Pose target, double thresholdInches) {
        Pose p = follower.getPose();
        double dx = p.getX() - target.getX();
        double dy = p.getY() - target.getY();
        return Math.sqrt(dx * dx + dy * dy) < thresholdInches;
    }

    private boolean arrived() {
        return !follower.isBusy();
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
    //comment
    public void adjustRotator(double tx) {
        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        // Blue side uses +offset instead of -offset
        int newPosition = getRotatorCurrentPositionCompat() + adjustment + offset;
        setRotatorTargetPosition(newPosition);
    }

    public void adjustHoodBasedOnDistance(double distance) {
        if (distance > DISTANCE_THRESHOLD) {
            setHoodPosition(FAR_HOOD_POSITION);
        } else {
            setHoodPosition(CLOSE_HOOD_POSITION);
        }
    }

    /**
     * Updates limelight-based adjustments (rotator, velocity, hood) during shooting states
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

                // Calculate distance and store it
                double currentDistance = getDist(tyDeg);
                if (currentDistance > 0) {
                    lastValidDistance = currentDistance;
                    hasValidLimelightData = true;

                    // Adjust rotator based on horizontal offset
                    adjustRotator(txDeg);

                    // Update velocity and hood based on distance
                    launcher.setVelocity(calcVelocity(currentDistance));
                    adjustHoodBasedOnDistance(currentDistance);
                }
            } else {
                // Limelight doesn't see target - use last valid values if available
                if (hasValidLimelightData) {
                    // Use last known good values (from previous successful detection)
                    adjustRotator(lastValidTx);
                    if (lastValidDistance > 0) {
                        launcher.setVelocity(calcVelocity(lastValidDistance));
                        adjustHoodBasedOnDistance(lastValidDistance);
                    }
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

    /** Pose-based rotator aim at the shared blue goal target. */
    private void aimRotatorAtBlueGoal() {
        if (rotator == null || follower == null) return;
        Pose robotPose = follower.getPose();
        if (robotPose == null) return;

        Pose goal = BlueUniversalConstants.target;
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngleDeg = angleToGoalDeg - Math.toDegrees(robotPose.getHeading());
        while (turretAngleDeg > 180) turretAngleDeg -= 360;
        while (turretAngleDeg < -180) turretAngleDeg += 360;

        // Match ChassisLocal.calculateTurretAngle sign convention (negative of turretAngle).
        double turretCmdDeg = -turretAngleDeg;
        double fracOf180 = Math.toRadians(turretCmdDeg) / Math.PI;
        int targetTicks = (int) (fracOf180 * motor180Range) + offset;
        setRotatorTargetPosition(targetTicks);
    }

    private void setRotatorTargetPosition(int ticks) {
        if (turretSubsystem != null) {
            turretSubsystem.setRotatorPos(ticks);
        } else if (rotator != null) {
            rotator.setTargetPosition(ticks);
        }
    }

    private int getRotatorCurrentPositionCompat() {
        //if (turretSubsystem != null) return turretSubsystem.getRotatorPos();
        if (rotator != null) return rotator.getCurrentPosition();
        return 0;
    }

    private void setHoodPosition(double pos) {
        if (turretSubsystem != null) {
            turretSubsystem.setHoodPos(pos);
        } else if (hood != null) {
            hood.setPosition(pos);
        }
    }

    private double getLauncherVelocityCompat() {
        if (turretSubsystem != null) {
            double[] v = turretSubsystem.getVelocity();
            return (v[0] + v[1]) / 2.0;
        }
        return launcher != null ? launcher.getVelocity() : 0.0;
    }

}
