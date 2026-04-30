//package org.firstinspires.ftc.teamcode.NewBotAuton;
//
//import com.pedropathing.follower.Follower;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
//import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
//import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;
//import com.pedropathing.math.Vector;
//import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
//import org.firstinspires.ftc.teamcode.robotControl.Subsystems.LookUpTables.ShotTimeLookupTable;
//import org.firstinspires.ftc.teamcode.robotControl.Subsystems.LookUpTables.VelocityLookupTable;
//import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
//import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
//import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;
//import org.firstinspires.ftc.teamcode.tests.ColorTesting;
//import org.firstinspires.ftc.teamcode.util.PoseStorage;
//import org.firstinspires.ftc.teamcode.LimeLight.BallLimelight;
//
//@Autonomous(name = "AutoTest", group = "auto", preselectTeleOp = "AmazingBotBlue")
//public class AutoTest extends OpMode {
//    private static final int PIPELINENUM = 0;
//    private static final int BALL_DETECTOR_PIPELINE = 1;
//    Pose target = new Pose(0, 144, Math.toRadians(144));
//    Pose sillyTarget;
//    private int rotatorStartPosition = 0;
//    double txDeg = 0.0;
//    double tyDeg = 0.0;
//    private Follower follower;
//    private BallLimelight ballLimelight;
//    /** 0 = following collect1, 1 = detour out, 2 = detour back, 3 = resume to collect pose */
//    private int ballDetourStep = 0;
//    private boolean ballDetourConsumed = false;
//    private Pose ballDetourOrigin;
//    private static final double BALL_DETOUR_RANGE_IN = 30.0;
//    private static final double BALL_DETOUR_MAX_LATERAL_IN = 18.0;
//    private static final double BALL_DETOUR_FORWARD_IN = 6.0;
//    private static final double BALL_DETOUR_MIN_DIST_TO_GOAL_IN = 16.0;
//
//    Robot robot;
//
//    private Vision vision = null;
//    private ColorTesting colorTesting = null;
//    private Intake intakeSubsystem = null;
//    private Turret turretSubsystem = null;
//    private TransferGate gateSubsystem = null;
//    private static final double BALL_DETECT_TIMEOUT_SEC = 3.0;
//
//    private boolean shoot1Started = false;
//    private boolean shoot2Started = false;
//    private boolean shoot3Started = false;
//    private boolean shoot4Started = false;
//    private boolean shoot5Started = false;
//    private boolean goAwayFromGateStarted = false;
//    private boolean goTowardsGateStarted = false;
//    private boolean opengateStarted = false;
//    private boolean parkingStarted = false;
//    private boolean collectionStarted = false;
//    private boolean beginGateCollectionStarted = false;
//    private boolean gateCollectionStarted = false;
//    private boolean turn = false;
//    private boolean beginGateCollectionAgainStarted = false;
//    private boolean gateCollectionAgainStarted = false;
//    private boolean shotFeeding = false;
//    private boolean gateWallDwellStarted = false;
//
//    private Servo hood, blocker;
//    private int limeHeight = 33;
//    private int offset = 28;
//    private int tagHeight = 75;
//    private static final double NORMAL_DRIVE_POWER = 1;
//    private static final double INTAKE_DRIVE_POWER = 1;
//    RobotActions actions;
//
//    private static final double DISTANCE_THRESHOLD = 180.0;
//    private static final double CLOSE_HOOD_POSITION = .2541;
//    private static final double FAR_HOOD_POSITION = 0.36;
//
//    private int y = tagHeight - limeHeight;
//
//    int motor180Range = 910;
//    int limelightUpAngle = 25;
//    private int vMultiplier = 9;
//    private Limelight3A limelight;
//
//    private double lastValidTx = 0.0;
//    private double lastValidTy = 0.0;
//    private double lastValidDistance = 0.0;
//    private boolean hasValidLimelightData = false;
//
//    private DcMotor leftFront, leftRear, rightFront, rightRear;
//
//    private DcMotorEx launcher;
//    private DcMotor tree, theWheelOfTheOx, rotator;
//    private Timer pathTimer, opModeTimer, gateTimer, shootTimer, openTimer;
//
//    public enum PathState {
//        start,
//        actuallyshoot1,
//        gotocollect,
//        collection,
//        goAwayFromGate,
//        goTowardsGate,
//        opengate,
//        shoot,
//        collectAgain,
//        collectAgainEnd,
//        shootAgain,
//        collectAgainAgain,
//        collectAgainAgainEnd,
//        shootAgainAgain,
//        collectAgainAgainAgain,
//        collectAgainAgainAgainEnd,
//        shootAgainAgainAgain,
//        parklol,
//        done,
//        VERYYYY_THIRD_INTAKE,
//        THIRD_SHOT_PREP,
//        PAUSE3,
//        Turny,
//        SHOT_3,
//        JACK_OFF,
//        beginGateCollection,
//        GateCollection,
//        beginGateCollectionAgain,
//        GateCollectionAgain,
//        GateCollectionAgainAgain,
//        shootAgainAgainAgainAgain,
//        shoot3ToGate,
//        LimelightSensorTracking
//    }
//
//    PathState pathState;
//
//    private final Pose startPose = new Pose(54.7, 7.6, Math.toRadians(180));
//    private final Pose shootPose1 = new Pose(54.7, 18, Math.toRadians(180));
//    private final Pose collect1thing = new Pose(13.5, 34, Math.toRadians(180));
//    private final Pose goToCollect1ControlPoint = new Pose(42.4576341127923, 35.30371389270978);
//    private final Pose shootPose2 = new Pose(47, 9, Math.toRadians(180));
//    private final Pose randomCollect = new Pose(9.5, 8, Math.toRadians(180));
//    private final Pose option1 = new Pose(13, 61, Math.toRadians(135));
//    private final Pose option2 = new Pose(7, 55, Math.toRadians(90));
//
//    private final Pose shootBall3 = new Pose(47, 9, Math.toRadians(180));
//    private final Pose inBetween2 = new Pose(44, 62, Math.toRadians(337.5));
//    private final Pose gateCollect2 = new Pose(15.5, 62, Math.toRadians(335));
//    private final Pose shootBall4 = new Pose(60, 75, Math.toRadians(310));
//    private final Pose gateCollect3 = new Pose(15.5, 62, Math.toRadians(335));
//    private final Pose shootBall5 = new Pose(49, 87, Math.toRadians(310));
//    private final Pose collect3end = new Pose(37, 8.8, Math.toRadians(180));
//    private final Pose collect3ControlPoint = new Pose(41.02, 84.857);
//    private final Pose shootBall6 = new Pose(55, 120, Math.toRadians(147));
//    private final Pose park = new Pose(41, 84, Math.toRadians(314));
//
//    private PathChain shoot1, Turn, goToCollect1, shoot3ToGate, collect1, shoot2, GateCollect3, shoot6, InBetween1, InBetween2, GateCollect1, GateCollect2, shoot3, awayfromGate, goToCollect3, collect3, shoot4, goToGate, openGate, goToCollect4, collect4, shoot5, parking;
//
//    public void buildPaths() {
//        shoot1 = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, shootPose1))
//                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
//                .build();
//
//        collect1 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose1, goToCollect1ControlPoint, collect1thing))
//                .setLinearHeadingInterpolation(shootPose1.getHeading(), collect1thing.getHeading())
//                .build();
//
//        shoot2 = follower.pathBuilder()
//                .addPath(new BezierLine(collect1thing, shootPose2))
//                .setLinearHeadingInterpolation(collect1thing.getHeading(), shootPose2.getHeading())
//                .build();
//        GateCollect1 = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose2, randomCollect))
//                .setLinearHeadingInterpolation(shootPose2.getHeading(), randomCollect.getHeading())
//                .build();
//        Turn = follower.pathBuilder()
//                .addPath(new BezierLine(randomCollect, option1))
//                .setLinearHeadingInterpolation(randomCollect.getHeading(), option1.getHeading())
//                .build();
//
//        shoot3 = follower.pathBuilder()
//                .addPath(new BezierLine(randomCollect, shootBall3))
//                .setLinearHeadingInterpolation(randomCollect.getHeading(), shootBall3.getHeading())
//                .build();
//
//        collect3 = follower.pathBuilder()
//                .addPath(new BezierLine(shootBall3, collect3end))
//                .setLinearHeadingInterpolation(shootBall3.getHeading(), collect3end.getHeading())
//                .build();
//
//        shoot6 = follower.pathBuilder()
//                .addPath(new BezierLine(collect3end, shootBall6))
//                .setLinearHeadingInterpolation(collect3end.getHeading(), shootBall6.getHeading())
//                .build();
//    }
//
//    public void statePathUpdate() {
//        switch (pathState) {
//            case start:
//                if (!shoot1Started) {
//                    robot.gate.block();
//                    robot.intake.powerON();
//                    follower.setMaxPower(NORMAL_DRIVE_POWER);
//                    follower.followPath(shoot1);
//                    shoot1Started = true;
//                }
//                if (shoot1Started) {
//                    robot.gate.open();
//                    pathState = PathState.actuallyshoot1;
//                    pathTimer.resetTimer();
//                }
//
//                break;
//            case actuallyshoot1:
//                robot.intake.shift();
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.25) {
//                    setPathState(PathState.collection);
//                }
//                break;
//
//            case collection:
//                if (!collectionStarted) {
//                    robot.gate.block();
//                    follower.followPath(collect1);
//                    collectionStarted = true;
//                    shotFeeding = false;
//                }
//                if ((arrived() || isNear(collect1thing, 3)) && collectionStarted) {
//                    setPathState((AutoTest.PathState.shoot));
//                }
//                break;
//            case shoot:
//                if (!shoot2Started) {
//                    follower.followPath(shoot2);
//                    shoot2Started = true;
//                }
//                if (shoot2Started && (arrived() || isNear(shootPose2, 5))) {
//                    robot.intake.powerON();
//                    if (!shotFeeding) {
//                        robot.gate.open();
//                        shootTimer.resetTimer();
//                        shotFeeding = true;
//                    }
//                    if (shotFeeding && shootTimer.getElapsedTimeSeconds() > 1.5) {
//                        setPathState((PathState.LimelightSensorTracking));
//                    }
//                }
//                break;
//            case LimelightSensorTracking:
//                if (!gateCollectionStarted) {
//                    robot.intake.down();
//                    robot.gate.block();
//                    follower.followPath(GateCollect1);
//                    gateCollectionStarted = true;
//                    openTimer.resetTimer();
//                }
//                boolean farEnoughForBallDetour = !isNear(randomCollect, BALL_DETOUR_MIN_DIST_TO_GOAL_IN);
//                if (!ballDetourConsumed
//                        && ballDetourStep == 0
//                        && ballLimelight.isConnected()
//                        && ballLimelight.seesBall()
//                        && farEnoughForBallDetour
//                        && follower.isBusy()) {
//                    ballDetourOrigin = follower.getPose();
//                    double tx = ballLimelight.getTxDegrees();
//                    // Positive tx = target right of crosshair → drive robot-right → negative robot-left.
//                    double lateral = -Math.tan(Math.toRadians(tx)) * BALL_DETOUR_RANGE_IN;
//                    lateral = Math.max(-BALL_DETOUR_MAX_LATERAL_IN,
//                            Math.min(BALL_DETOUR_MAX_LATERAL_IN, lateral));
//                    Pose toward = offsetRobotRelative(ballDetourOrigin, BALL_DETOUR_FORWARD_IN, lateral);
//                    PathChain detourOut = follower.pathBuilder()
//                            .addPath(new BezierLine(ballDetourOrigin, toward))
//                            .setLinearHeadingInterpolation(
//                                    ballDetourOrigin.getHeading(), toward.getHeading())
//                            .build();
//                    follower.followPath(detourOut);
//                    ballDetourStep = 1;
//                    break;
//                }
//                if (ballDetourStep == 1 && arrived()) {
//                    Pose at = follower.getPose();
//                    PathChain detourBack = follower.pathBuilder()
//                            .addPath(new BezierLine(at, ballDetourOrigin))
//                            .setLinearHeadingInterpolation(at.getHeading(), ballDetourOrigin.getHeading())
//                            .build();
//                    follower.followPath(detourBack);
//                    ballDetourStep = 2;
//                    break;
//                }
//                if (ballDetourStep == 2 && arrived()) {
//                    Pose at2 = follower.getPose();
//                    PathChain resumeToCollect = follower.pathBuilder()
//                            .addPath(new BezierLine(at2, randomCollect))
//                            .setLinearHeadingInterpolation(at2.getHeading(), randomCollect.getHeading())
//                            .build();
//                    follower.followPath(resumeToCollect);
//                    ballDetourStep = 3;
//                    break;
//                }
//                if (ballDetourStep == 3 && arrived()) {
//                    ballDetourStep = 0;
//                    ballDetourConsumed = true;
//                }
//                Intake.DetectedColor color = robot.intake.AutonColor(telemetry);
//                boolean full = (color != Intake.DetectedColor.UNKNOWN);
//                boolean ready = full || openTimer.getElapsedTimeSeconds() > 1.0;
//                boolean forceGateLeave = opModeTimer.getElapsedTimeSeconds() >= 27.5;
//
//                if (gateCollectionStarted
//                        && (((arrived() || isNear(randomCollect, 3)) && ready) || forceGateLeave)) {
//                    robot.intake.down();
//                    setPathState((AutoTest.PathState.shootAgain));
//                }
//                break;
//            case shootAgain:
//                if (!shoot3Started) {
//                    follower.followPath(shoot3);
//                    shoot3Started = true;
//                }
//                if (shoot3Started && (arrived() || isNear(shootBall3, 5))) {
//                    robot.intake.powerON();
//                    if (!shotFeeding) {
//                        robot.gate.open();
//                        shootTimer.resetTimer();
//                        shotFeeding = true;
//                    }
//                    boolean readyToLeave = shotFeeding && shootTimer.getElapsedTimeSeconds() > 1.5;
//                    if (readyToLeave) {
//                        if (opModeTimer.getElapsedTimeSeconds() < 27) {
//                            setPathState((AutoTest.PathState.GateCollection));
//                        } else {
//                            setPathState(PathState.collectAgainAgainEnd);
//                        }
//                    }
//                }
//                break;
//            case collectAgainAgainEnd:
//                robot.gate.block();
//                if (!collectionStarted) {
//                    follower.followPath(collect3);
//                    collectionStarted = true;
//                }
//                if ((arrived() || isNear(collect3end, 3)) && collectionStarted) {
//                    setPathState(PathState.done);
//                }
//                break;
//            case done:
//                break;
//        }
//    }
//
//    public void setPathState(PathState newState) {
//        pathState = newState;
//        pathTimer.resetTimer();
//        shoot1Started = false;
//        shoot2Started = false;
//        shoot3Started = false;
//        shoot4Started = false;
//        shoot5Started = false;
//        goAwayFromGateStarted = false;
//        goTowardsGateStarted = false;
//        opengateStarted = false;
//        parkingStarted = false;
//        collectionStarted = false;
//        beginGateCollectionStarted = false;
//        turn = false;
//        gateCollectionStarted = false;
//        beginGateCollectionAgainStarted = false;
//        gateCollectionAgainStarted = false;
//        shotFeeding = false;
//        gateWallDwellStarted = false;
//        ballDetourStep = 0;
//        ballDetourConsumed = false;
//    }
//
//    @Override
//    public void init() {
//        pathState = PathState.start;
//
//        pathTimer = new Timer();
//        opModeTimer = new Timer();
//        gateTimer = new Timer();
//        shootTimer = new Timer();
//        openTimer = new Timer();
//
//        follower = ConstantsNewBot.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        buildPaths();
//
//        ballLimelight = new BallLimelight(hardwareMap);
//        if (ballLimelight.isConnected() && ballLimelight.getCamera() != null) {
//            ballLimelight.getCamera().pipelineSwitch(BALL_DETECTOR_PIPELINE);
//        }
//
//        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
//        intakeSubsystem = robot.intake;
//        turretSubsystem = robot.turret;
//        gateSubsystem = robot.gate;
//        actions = new RobotActions(
//                robot.chassisLocal,
//                robot.vision,
//                robot.turret,
//                robot.gate,
//                robot.intake);
//        robot.intake.down();
//        robot.gate.block();
//
//        sillyTarget = robot.chassisLocal.sillyTargetPose(target);
//
//        telemetry.addLine("Good to go BLUE");
//        telemetry.addData("Ball LL pipeline", BALL_DETECTOR_PIPELINE);
//        telemetry.update();
//    }
//
//    public void start() {
//        opModeTimer.resetTimer();
//        setPathState(pathState);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        // ChassisLocal owns a separate Follower instance; sync pose so RobotActions distance matches pathing.
//        robot.chassisLocal.setPose(follower.getPose());
//        statePathUpdate();
//
//        Pose currentPose = follower.getPose();
//        PoseStorage.savePose(currentPose);
//
//        double dx = target.getX() - currentPose.getX();
//        double dy = target.getY() - currentPose.getY();
//        double dist = Math.sqrt(dx * dx + dy * dy);
//
//        Vector velocity = follower.getVelocity();
//        double vx = velocity.getMagnitude() * Math.cos(velocity.getTheta());
//        double vy = velocity.getMagnitude() * Math.sin(velocity.getTheta());
//        double shotTime = ShotTimeLookupTable.getTime(dist);
//        Pose aimTarget;
//        if (dist < 120) {
//            aimTarget = new Pose(
//                    target.getX() - vx * shotTime,
//                    target.getY() - vy * shotTime
//            );
//        } else {
//            aimTarget = target;
//        }
//
//        double aimDx = aimTarget.getX() - currentPose.getX();
//        double aimDy = aimTarget.getY() - currentPose.getY();
//        double distForShooter = Math.hypot(aimDx, aimDy);
//        double angleToGoal = Math.toDegrees(Math.atan2(aimDy, aimDx));
//        double turretAngle = angleToGoal - Math.toDegrees(currentPose.getHeading());
//        while (turretAngle > 180) turretAngle -= 360;
//        while (turretAngle < -180) turretAngle += 360;
//        robot.turret.setRotatorToAngle(-turretAngle);
//
//        // Quadratic RPM vs distance (and RPM→ticks) from RobotActions; hood set here by zone overwrites equation hood.
//        actions.autoVelocityEquation(aimTarget);
//        int zone = VelocityLookupTable.getZone(distForShooter);
//        if (zone == 1) robot.turret.setHoodPos(BlueUniversalConstants.CLOSE_HOOD_POSITION);
//        else if (zone == 2) robot.turret.setHoodPos(BlueUniversalConstants.MID_HOOD_POSITION);
//        else robot.turret.setHoodPos(BlueUniversalConstants.FAR_HOOD_POSITION);
//
//        telemetry.addData("paths state", pathState.toString());
//        telemetry.addData("x", currentPose.getX());
//        telemetry.addData("y", currentPose.getY());
//        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
//        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
//        telemetry.addData("Dist to goal", dist);
//        telemetry.addData("Dist for shooter (lead)", distForShooter);
//        telemetry.addData("Turret Angle", -turretAngle);
//        telemetry.addData("Launcher velocity", robot.turret.getTargetVelocity());
//        telemetry.addData("Ball LL", ballLimelight.isConnected());
//        telemetry.addData("Sees ball", ballLimelight.isConnected() && ballLimelight.seesBall());
//        telemetry.addData("Ball detour step", ballDetourStep);
//        telemetry.addData("Ball detour used", ballDetourConsumed);
//    }
//
//    private boolean shootinAuton() {
//        Pose pos = follower.getPose();
//        double y = pos.getY();
//        double x = pos.getX();
//        double leftUpBound = -x + 144;
//        double leftBottomBound = x - 48;
//        double rightUpBound = x;
//        double rightBottomBound = -x + 144 - 48;
//        if (x < 72) {
//            return (y > leftUpBound || y < leftBottomBound);
//        } else {
//            return (y > rightUpBound || y < rightBottomBound);
//        }
//    }
//
//    private boolean isNear(Pose target, double thresholdInches) {
//        Pose p = follower.getPose();
//        double dx = p.getX() - target.getX();
//        double dy = p.getY() - target.getY();
//        return Math.sqrt(dx * dx + dy * dy) < thresholdInches;
//    }
//
//    private boolean arrived() {
//        return !follower.isBusy();
//    }
//
//    /** Robot-relative offset: forward along heading, positive left perpendicular to heading. */
//    private static Pose offsetRobotRelative(Pose p, double forwardIn, double leftIn) {
//        double h = p.getHeading();
//        double cos = Math.cos(h);
//        double sin = Math.sin(h);
//        double x = p.getX() + forwardIn * cos + leftIn * (-sin);
//        double y = p.getY() + forwardIn * sin + leftIn * cos;
//        return new Pose(x, y, h);
//    }
//}
