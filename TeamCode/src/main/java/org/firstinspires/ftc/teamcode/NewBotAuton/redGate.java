package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotControl.RedUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;

import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.LookUpTables.VelocityLookupTable;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TestTurret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.tests.ColorTesting;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "redGate", group = "auto", preselectTeleOp = "Red Tele")
public class redGate extends OpMode implements RedUniversalConstants {
    /**
     * Fixed field-relative turret angle (degrees) for this auton after run starts.
     * Tune on the field (telemetry used to show dynamic angle; match that here).
     */
    private static final double AUTON_FIXED_TURRET_ANGLE_DEG = -20;
    private static final double AUTON_FIXED_TURRET_ANGLE_DEG_FIRST = -22;
    private static final double AUTON_FIXED_TURRET_ANGLE_DEG_FINAL = 7;
    private static final double AUTON_RPM_OFFSET = 20;
    private static final double AUTON_RPM_OFFSET_FIRST_SHOT = 100;

    private static final double AUTON_RPM_OFFSET_LAST_SHOT = 0;
    /** Open blocker (`robot.gate`) when this close to the shot pose (inches). */
    private static final double GATE_OPEN_PROXIMITY_IN = 3;
    private static final double FIRST_SHOT_PROXIMITY = 5;
    private static final double LAST_SHOT_PROXIMITY = 35;


    Pose sillyTarget;
    TestTurret testTurret;
    private int rotatorStartPosition=0;
    double txDeg = 0.0;
    double tyDeg = 0.0;
    Robot robot;
    private RobotActions actions;
    private Vision vision = null;
    private ColorTesting colorTesting = null;
    private Intake intakeSubsystem = null;
    private TransferGate gateSubsystem = null;
    private static final double BALL_DETECT_TIMEOUT_SEC = 3;

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
    private boolean turn = false;
    private boolean beginGateCollectionAgainStarted = false;
    private boolean gateCollectionAgainStarted = false;
    private boolean shotFeeding = false;
    /** True after we have reached the gate pose; gate dwell timer runs only from here. */
    private boolean gateWallDwellStarted = false;

    private Servo hood, blocker;
    private int limeHeight = 33;
    private int offset = 28;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 1;

    private static final double DISTANCE_THRESHOLD = 180.0;
    private static final double CLOSE_HOOD_POSITION = .2541;
    private static final double FAR_HOOD_POSITION = 0.36;

    private int y = tagHeight - limeHeight;

    int motor180Range = 910;
    int limelightUpAngle = 25;
    private int vMultiplier = 9;
    private Limelight3A limelight;

    private double lastValidTx = 0.0;
    private double lastValidTy = 0.0;
    private double lastValidDistance = 0.0;
    private boolean hasValidLimelightData = false;

    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private DcMotorEx launcher;
    private DcMotor tree, theWheelOfTheOx, rotator;
    private Timer pathTimer, opModeTimer, gateTimer, shootTimer, openTimer;

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
        Turny,
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

    private final Pose startPose = new Pose(112.217235446, 136.97551020408164, Math.toRadians(-90));
    private final Pose shootPose1 = new Pose(81, 77, Math.toRadians(25));
    private final Pose collect1thing = new Pose(134, 62, Math.toRadians(0));
    private final Pose goToCollect1ControlPoint = new Pose(91.735, 60.091);
    private final Pose shootPose2 = new Pose(81, 77, Math.toRadians(25));
    private final Pose gateCollect1 = new Pose(131.5, 62, Math.toRadians(32));
    private final Pose option1 = new Pose(131, 61, Math.toRadians(45));
    private final Pose option2 = new Pose(137, 55, Math.toRadians(90));

    private final Pose shootBall3 = new Pose(81, 77, Math.toRadians(25));
    private final Pose inBetween2 = new Pose(100, 62, Math.toRadians(202.5));
    private final Pose gateCollect2 = new Pose(128.5, 62, Math.toRadians(205));
    private final Pose shootBall4 = new Pose(84, 75, Math.toRadians(230));
    private final Pose gateCollect3 = new Pose(128.5, 62, Math.toRadians(205));
    private final Pose shootBall5 = new Pose(95, 87, Math.toRadians(230));
    private final Pose collect3end = new Pose(126, 88, Math.toRadians(0));
    private final Pose collect3ControlPoint = new Pose(102.98, 84.857);
    private final Pose shootBall6 = new Pose(89, 120, Math.toRadians(33));
    private final Pose park = new Pose(103, 84, Math.toRadians(226));

    private PathChain shoot1, Turn, goToCollect1, shoot3ToGate, collect1, shoot2, GateCollect3, shoot6, InBetween1, InBetween2, GateCollect1, GateCollect2, shoot3, awayfromGate, goToCollect3, collect3, shoot4, goToGate, openGate, goToCollect4, collect4, shoot5, parking;

    public void buildPaths() {
        Follower follower = robot.chassisLocal.getFollower();
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        collect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, goToCollect1ControlPoint, collect1thing))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), collect1thing.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1thing, shootPose2))
                .setLinearHeadingInterpolation(collect1thing.getHeading(), shootPose2.getHeading())
                .build();
        GateCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, gateCollect1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gateCollect1.getHeading())
                .build();
        Turn = follower.pathBuilder()
                .addPath(new BezierLine(gateCollect1, option1))
                .setLinearHeadingInterpolation(gateCollect1.getHeading(), option1.getHeading())
                .build();


        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(gateCollect1, shootBall3))
                .setLinearHeadingInterpolation(gateCollect1.getHeading(), shootBall3.getHeading())
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new BezierLine(shootBall3, collect3end))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), collect3end.getHeading())
                .build();

        shoot6 = follower.pathBuilder()
                .addPath(new BezierLine(collect3end, shootBall6))
                .setLinearHeadingInterpolation(collect3end.getHeading(), shootBall6.getHeading())
                .build();
    }

    public void statePathUpdate() {
        Follower follower = robot.chassisLocal.getFollower();
        switch (pathState) {
            case start:
                if (!shoot1Started) {
                    robot.gate.block();
                    robot.intake.powerON();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(shoot1);
                    shoot1Started = true;
                }
                // Do not use setPathState here: it clears shoot1Started and breaks the first-shot logic.
                if (shoot1Started) {
                    pathState = PathState.actuallyshoot1;
                    pathTimer.resetTimer();
                }

                break;
            case actuallyshoot1:
                robot.intake.shift();
                // Open gate when close to first shot pose (do not clear shotFeeding every frame — that
                // was resetting the feed timer every loop so collection never triggered).
                if (isNear(shootPose1, FIRST_SHOT_PROXIMITY)) {
                    robot.gate.open();
                }
                if (shoot1Started && (arrived() || isNear(shootPose1, FIRST_SHOT_PROXIMITY))) {
                    if (!shotFeeding) {
                        robot.gate.open();
                        shootTimer.resetTimer();
                        shotFeeding = true;
                    }
                    if (shotFeeding && shootTimer.getElapsedTimeSeconds() > 0.5) {
                        setPathState(PathState.collection);
                    }
                }
                break;

            case collection:
                if (!collectionStarted) {
                    robot.gate.block();
                    follower.followPath(collect1);
                    collectionStarted = true;
                    shotFeeding = false;
                }
                if ((arrived() || isNear(collect1thing, 3)) && collectionStarted) {
                    setPathState((redGate.PathState.shoot));
                }
                break;
            case shoot:
                if (!shoot2Started) {
                    follower.followPath(shoot2);
                    shoot2Started = true;
                }
                if (shoot2Started && (arrived() || isNear(shootPose2, GATE_OPEN_PROXIMITY_IN))) {
                    robot.intake.powerON();
                    if (!shotFeeding) {
                        robot.gate.open();
                        shootTimer.resetTimer();
                        shotFeeding = true;
                    }
                    if (shotFeeding && shootTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intake.gateCollet();
                        setPathState((PathState.GateCollection));
                    }
                }
                break;
            case GateCollection:
                if (!gateCollectionStarted) {
                    robot.intake.gateCollet();
                    robot.gate.block();
                    follower.followPath(GateCollect1);
                    gateCollectionStarted = true;
                    openTimer.resetTimer();
                }
                Intake.DetectedColor color = robot.intake.getDetectedColorByDistance(telemetry);
                robot.intake.gateCollet();
                boolean full = (color != Intake.DetectedColor.UNKNOWN);
                boolean ready = full || openTimer.getElapsedTimeSeconds() > 3.3;
                boolean forceGateLeave = opModeTimer.getElapsedTimeSeconds() >= 25.5;

                if (gateCollectionStarted
                        && (((arrived() || isNear(gateCollect1, 5)) && ready) || forceGateLeave)) {
                    robot.intake.up();
                    robot.gate.block();
                    shotFeeding = false;
                    setPathState((redGate.PathState.shootAgain));
                }
                break;
//            case Turny:
//                if (!turn) {2
//                    robot.intake.shift();
//                    robot.gate.block();
//                    follower.followPath(Turn);
//                    turn = true;
//                    gateTimer.resetTimer();
//                }
//                Intake.DetectedColor color = robot.intake.AutonColor(telemetry);
//                boolean full = (color != Intake.DetectedColor.UNKNOWN);
//                boolean ready = full || gateTimer.getElapsedTimeSeconds() >5;
//
//                if ((arrived() || isNear(option1, 3)) && turn && ready) {
//                    robot.intake.down();
//                    setPathState((test2.PathState.shootAgain));
//                }
//                break;
            case shootAgain:
                if (!shoot3Started) {
                    follower.followPath(shoot3);
                    shoot3Started = true;
                }
                if (shoot3Started && (arrived() || isNear(shootBall3, GATE_OPEN_PROXIMITY_IN))) {
                    robot.intake.powerON();
                    if (!shotFeeding) {
                        shootTimer.resetTimer();
                        shotFeeding = true;
                    }
                    if (isNear(shootBall3, GATE_OPEN_PROXIMITY_IN)) {
                        robot.gate.open();
                    }
                    boolean readyToLeave = shotFeeding && shootTimer.getElapsedTimeSeconds() > 0.45;
                    if (readyToLeave) {
                        if (opModeTimer.getElapsedTimeSeconds() < 25) {
                            setPathState((redGate.PathState.GateCollection));
                        } else {
                            robot.intake.shift();
                            setPathState(PathState.collectAgainAgainEnd);
                        }
                    }
                }
                break;
            case collectAgainAgainEnd:
                robot.intake.shift();
                robot.gate.block();
                if (!collectionStarted) {
                    follower.followPath(collect3);
                    collectionStarted = true;
                }
                if ((arrived() || isNear(collect3end, 3)) && collectionStarted) {
                    setPathState((redGate.PathState.shootAgainAgainAgainAgain));
                }
                break;
            case shootAgainAgainAgainAgain:
                if (!shoot5Started) {
                    follower.followPath(shoot6);
                    shoot5Started = true;
                }
                if (shoot5Started && (arrived() || isNear(shootBall6, LAST_SHOT_PROXIMITY))) {
                    robot.intake.powerON();
                    if (!shotFeeding) {
                        robot.gate.open();
                        shootTimer.resetTimer();
                        shotFeeding = true;
                    }
                    if (shotFeeding && shootTimer.getElapsedTimeSeconds() > 0.5) {
                        setPathState((redGate.PathState.done));
                    }
                }
                break;
            case done:
                robot.turret.setRotatorPos(0.5);
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
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
        turn = false;
        gateCollectionStarted = false;
        beginGateCollectionAgainStarted = false;
        gateCollectionAgainStarted = false;
        shotFeeding = false;
        gateWallDwellStarted = false;
    }

    @Override
    public void init() {
        pathState = PathState.start;

        pathTimer = new Timer();
        opModeTimer = new Timer();
        gateTimer = new Timer();
        shootTimer = new Timer();
        openTimer = new Timer();

        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
        // Important: create TestTurret AFTER Robot so launcher motors end in TestTurret's
        // RUN_WITHOUT_ENCODER + manual power-loop configuration (same order as AmazingBotBlue).
        testTurret = new TestTurret(hardwareMap);
        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                testTurret,
                robot.gate,
                robot.intake
        );
        buildPaths();
        robot.intake.down();
        robot.gate.block();

        sillyTarget = autonTarget;

        telemetry.addLine("Good to go RED");
        telemetry.update();
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    private double getFixedTurretAngleDeg() {
        if (pathState == PathState.actuallyshoot1) {
            return AUTON_FIXED_TURRET_ANGLE_DEG_FIRST;
        }
        if (pathState == PathState.shootAgainAgainAgainAgain) {
            return AUTON_FIXED_TURRET_ANGLE_DEG_FINAL;
        }
        return AUTON_FIXED_TURRET_ANGLE_DEG;
    }

    private double getAutonRpmOffset() {
        if (pathState == PathState.actuallyshoot1) {
            return AUTON_RPM_OFFSET_FIRST_SHOT;
        }
        if (pathState == PathState.shootAgainAgainAgainAgain) {
            return AUTON_RPM_OFFSET;
        }
        return AUTON_RPM_OFFSET;
    }
    //
    @Override
    public void loop() {
        robot.chassisLocal.update();
        statePathUpdate();

        Pose currentPose = robot.chassisLocal.getPose();
        PoseStorage.savePose(currentPose);

        double dist = robot.chassisLocal.getDistance(autonTarget);
        sillyTarget = autonTarget;

        double fixedTurretAngleDeg = getFixedTurretAngleDeg();
        testTurret.setRotatorToAngle(fixedTurretAngleDeg);
        actions.autoVelocityEquation(autonTarget);
        testTurret.setTargetRPM(testTurret.getTargetVelocity() + getAutonRpmOffset());
        double distForShooter = dist;
        int zone = VelocityLookupTable.getZone(distForShooter);
        if (zone == 1) testTurret.setHoodPos(RedUniversalConstants.CLOSE_HOOD_POSITION);
        else if (zone == 2) testTurret.setHoodPos(RedUniversalConstants.MID_HOOD_POSITION);
        else testTurret.setHoodPos(RedUniversalConstants.FAR_HOOD_POSITION);
        testTurret.update(distForShooter);

        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", currentPose.getX());
        telemetry.addData("y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Dist to goal", dist);
        telemetry.addData("Dist for shooter", distForShooter);
        telemetry.addData("Turret Angle (fixed)", fixedTurretAngleDeg);
        telemetry.addData("RPM actual", testTurret.getRPM());
        telemetry.addData("RPM target", testTurret.getTargetVelocity());
        telemetry.update();
    }

    private boolean shootinAuton() {
        Pose pos = robot.chassisLocal.getPose();
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
        Pose p = robot.chassisLocal.getPose();
        double dx = p.getX() - target.getX();
        double dy = p.getY() - target.getY();
        return Math.sqrt(dx * dx + dy * dy) < thresholdInches;
    }

    private boolean arrived() {
        return robot.chassisLocal.getFollower().atParametricEnd();
    }
}
