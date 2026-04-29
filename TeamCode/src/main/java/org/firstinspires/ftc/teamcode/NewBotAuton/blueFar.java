package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.pedropathing.follower.Follower;
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;

import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.LookUpTables.VelocityLookupTable;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.tests.ColorTesting;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "blueFar", group = "auto", preselectTeleOp = "AmazingBotBlue")
public class blueFar extends OpMode {
    private static final int PIPELINENUM = 0;
    private static final double FIRST_SHOT_PROXIMITY_IN = 5.0;
    private static final double FIRST_SHOT_SETTLE_SEC = 1.3;
    private static final double FIRST_SHOT_FEED_SEC = 0.5;
    private static final double AUTON_VELOCITY_OFFSET_TPS = 110;
    Pose target = new Pose(4, 144, Math.toRadians(144));
    Pose sillyTarget;
    private int rotatorStartPosition = 0;
    double txDeg = 0.0;
    double tyDeg = 0.0;
    Robot robot;

    private Vision vision = null;
    private ColorTesting colorTesting = null;
    private Intake intakeSubsystem = null;
    private Turret turretSubsystem = null;
    private TransferGate gateSubsystem = null;
    private static final double BALL_DETECT_TIMEOUT_SEC = 3.0;

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
    private boolean gateWallDwellStarted = false;

    private Servo hood, blocker;
    private int limeHeight = 33;
    private int offset = 28;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 1;
    RobotActions actions;

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

    private final Pose startPose = new Pose(54.7, 7.6, Math.toRadians(180));
    private final Pose shootPose1 = new Pose(54.7, 18, Math.toRadians(180));
    private final Pose collect1thing = new Pose(13.5, 34, Math.toRadians(180));
    private final Pose goToCollect1ControlPoint = new Pose(42.4576341127923, 35.30371389270978);
    private final Pose shootPose2 = new Pose(47, 10, Math.toRadians(180));
    private final Pose gateCollect1 = new Pose(9.5, 10, Math.toRadians(180));
    private final Pose option1 = new Pose(13, 61, Math.toRadians(135));
    private final Pose option2 = new Pose(7, 55, Math.toRadians(90));

    private final Pose shootBall3 = new Pose(47, 10, Math.toRadians(180));
    private final Pose inBetween2 = new Pose(44, 62, Math.toRadians(337.5));
    private final Pose gateCollect2 = new Pose(15.5, 62, Math.toRadians(335));
    private final Pose shootBall4 = new Pose(60, 75, Math.toRadians(310));
    private final Pose gateCollect3 = new Pose(15.5, 62, Math.toRadians(335));
    private final Pose shootBall5 = new Pose(49, 87, Math.toRadians(310));
    private final Pose collect3end = new Pose(37, 15, Math.toRadians(180));
    private final Pose collect3ControlPoint = new Pose(41.02, 84.857);
    private final Pose shootBall6 = new Pose(55, 120, Math.toRadians(147));
    private final Pose park = new Pose(41, 84, Math.toRadians(314));

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
        if (opModeTimer.getElapsedTimeSeconds() > 28.0
                && pathState != PathState.collectAgainAgainEnd
                && pathState != PathState.done) {
            setPathState(PathState.collectAgainAgainEnd);
        }
        switch (pathState) {
            case start:
                if (!shoot1Started) {
                    robot.gate.block();
                    robot.intake.powerON();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(shoot1);
                    shoot1Started = true;
                }
                if (shoot1Started) {
                    pathState = PathState.actuallyshoot1;
                    pathTimer.resetTimer();
                }

                break;
            case actuallyshoot1:
                robot.intake.shift();
                boolean atFirstShotPose = arrived() || isNear(shootPose1, FIRST_SHOT_PROXIMITY_IN);
                if (atFirstShotPose && !shotFeeding
                        && pathTimer.getElapsedTimeSeconds() > FIRST_SHOT_SETTLE_SEC) {
                    robot.gate.open();
                    shootTimer.resetTimer();
                    shotFeeding = true;
                }
                if (shotFeeding && shootTimer.getElapsedTimeSeconds() > FIRST_SHOT_FEED_SEC) {
                    setPathState(PathState.collection);
                }
                break;

            case collection:
                if (!collectionStarted) {
                    robot.gate.block();
                    follower.followPath(collect1);
                    collectionStarted = true;
                }
                if ((arrived() || isNear(collect1thing, 3)) && collectionStarted) {
                    setPathState((blueFar.PathState.shoot));
                }
                break;
            case shoot:
                if (!shoot2Started) {
                    follower.followPath(shoot2);
                    shoot2Started = true;
                }
                if (shoot2Started && (arrived() || isNear(shootPose2, 5))) {
                    robot.intake.powerON();
                    if (!shotFeeding) {
                        robot.gate.open();
                        shootTimer.resetTimer();
                        shotFeeding = true;
                    }
                    if (shotFeeding && shootTimer.getElapsedTimeSeconds() > 1.5) {
                        setPathState((PathState.GateCollection));
                    }
                }
                break;
            case GateCollection:
                robot.intake.shift();
                if (!gateCollectionStarted) {
                    robot.intake.shift();
                    robot.gate.block();
                    follower.followPath(GateCollect1);
                    gateCollectionStarted = true;
                    openTimer.resetTimer();
                }
                Intake.DetectedColor color = robot.intake.AutonColor(telemetry);
                boolean full = (color != Intake.DetectedColor.UNKNOWN);
                boolean ready = full || openTimer.getElapsedTimeSeconds() > 1.0;
                boolean forceGateLeave = opModeTimer.getElapsedTimeSeconds() >= 27.5;

                if (gateCollectionStarted
                        && (((arrived() || isNear(gateCollect1, 3)) && ready) || forceGateLeave)) {
                    robot.intake.up();
                    setPathState((blueFar.PathState.shootAgain));
                }
                break;
            case shootAgain:
                if (!shoot3Started) {
                    follower.followPath(shoot3);
                    shoot3Started = true;
                }
                if (shoot3Started && (arrived() || isNear(shootBall3, 5))) {
                    robot.intake.powerON();
                    if (!shotFeeding) {
                        robot.gate.open();
                        shootTimer.resetTimer();
                        shotFeeding = true;
                    }
                    boolean readyToLeave = shotFeeding && shootTimer.getElapsedTimeSeconds() > 1.5;
                    if (readyToLeave) {
                        if (opModeTimer.getElapsedTimeSeconds() < 28) {
                            setPathState((blueFar.PathState.GateCollection));
                        } else {
                            setPathState(PathState.collectAgainAgainEnd);
                        }
                    }
                }
                break;
            case collectAgainAgainEnd:
                robot.gate.block();
                if (!collectionStarted) {
                    follower.followPath(collect3);
                    collectionStarted = true;
                }
                if ((arrived() || isNear(collect3end, 3)) && collectionStarted) {
                    setPathState(PathState.done);
                }
                break;
            case done:
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
        intakeSubsystem = robot.intake;
        turretSubsystem = robot.turret;
        gateSubsystem = robot.gate;
        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                robot.turret,
                robot.gate,
                robot.intake);
        buildPaths();
        robot.intake.down();
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
        robot.chassisLocal.update();
        statePathUpdate();

        Pose currentPose = robot.chassisLocal.getPose();
        PoseStorage.savePose(currentPose);

        // Lock auton aiming target to fixed field point (4, 144) for all states.
        actions.setTargetPose(target);
        Pose activeTarget = target;
        actions.aimTurret(activeTarget);
        // Slightly reduce shooter speed for auton consistency.
        robot.turret.setVelocity(Math.max(0, robot.turret.getTargetVelocity() - AUTON_VELOCITY_OFFSET_TPS));

        double dist = robot.chassisLocal.getDistance(activeTarget);
        double distForShooter = dist;
        double turretAngle = robot.chassisLocal.calculateTurretAngle(activeTarget);

        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", currentPose.getX());
        telemetry.addData("y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Dist to goal", dist);
        telemetry.addData("Dist for shooter", distForShooter);
        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Launcher velocity", robot.turret.getTargetVelocity());
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
