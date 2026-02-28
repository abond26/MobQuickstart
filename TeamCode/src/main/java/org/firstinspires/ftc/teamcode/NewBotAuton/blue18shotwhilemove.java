package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.math.Vector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;
import org.firstinspires.ftc.teamcode.robotControl.ShotTimeLookupTable;
import org.firstinspires.ftc.teamcode.robotControl.VelocityLookupTable;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "Blue 18 close shot while move", group = "new bot")
public class blue18shotwhilemove extends OpMode {
    private int rotatorStartPosition = 0;
    private Follower follower;

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
    /** True once we've opened blocker for the current move-shot (so we only open once per shot). */
    private boolean moveShotBlockerOpened = false;

    private Servo hood, blocker;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 1;

    private static final double DISTANCE_THRESHOLD = 180.0;
    private static final double CLOSE_HOOD_POSITION = .2541;
    private static final double FAR_HOOD_POSITION = 0.36;

    int motor180Range = 910;
    private int offset = 28;

    private DcMotorEx launcher;
    private DcMotor tree, theWheelOfTheOx, rotator;
    private Timer pathTimer, opModeTimer;

    /** Blue backboard goal for aiming (field coords). */
    private static final double BLUE_GOAL_X = 1;
    private static final double BLUE_GOAL_Y = 144;
    /** Shoot line from (0,144) to (72,72): equation x+y=144. Open blocker when we're within this many (x+y)-units before the line (144 - x - y in (0, this]). */
    private static final double LEAD_BEFORE_SHOOT_LINE = 10.0;
    /** After opening blocker in move-shot, wait this long (sec) before transitioning to next state. */
    private static final double MOVE_SHOT_DWELL_SEC = 1.0;

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
        beginGateCollection,
        GateCollection,
        GateCollectionAgain,
        GateCollectionAgainAgain,
        GateCollectionAgainAgainAgain,
        shootAgainAgainAgainAgain,
    }

    PathState pathState;
    private final Pose startPose = new Pose(26.7, 132, Math.toRadians(144));
    private final Pose shootPose1 = new Pose(46, 97.5, Math.toRadians(134));
    private final Pose collect1thing = new Pose(19, 61, Math.toRadians(180));
    private final Pose goToCollect1ControlPoint = new Pose(65, 58.5, Math.toRadians(180));
    // Move-shot end poses: further along (higher x, higher y) so path extends past shoot line = more time for all 3 rings, then turn back to collect
    private static final double MOVE_SHOT_END_X = 58;
    private static final double MOVE_SHOT_END_Y = 119;
    private static final double MOVE_SHOT_END_HEADING_DEG = 132;
    private final Pose shootPose2 = new Pose(MOVE_SHOT_END_X, MOVE_SHOT_END_Y, Math.toRadians(MOVE_SHOT_END_HEADING_DEG));
    private final Pose shoot2ControlPoint1 = new Pose(49.15667574931882, 76, Math.toRadians(180));
    private final Pose gateCollect1 = new Pose(17, 62, Math.toRadians(150));
    private final Pose shootPose2ToGateControlPoint = new Pose(50, 55.801430517711175, Math.toRadians(180));
    private final Pose shootBall3 = new Pose(MOVE_SHOT_END_X, MOVE_SHOT_END_Y, Math.toRadians(MOVE_SHOT_END_HEADING_DEG));
    private final Pose gateCollect2 = new Pose(17, 62, Math.toRadians(150));
    private final Pose shootBall4 = new Pose(MOVE_SHOT_END_X, MOVE_SHOT_END_Y, Math.toRadians(MOVE_SHOT_END_HEADING_DEG));
    private final Pose gateCollect3 = new Pose(17, 62, Math.toRadians(150));
    private final Pose shootBall5 = new Pose(MOVE_SHOT_END_X, MOVE_SHOT_END_Y, Math.toRadians(MOVE_SHOT_END_HEADING_DEG));
    private final Pose shoot4ToCollect3ControlPoint = new Pose(41.25340599455039, 82.36784741144412, Math.toRadians(180));
    private final Pose collect3end = new Pose(25, 86, Math.toRadians(180));
    private final Pose shootBall6 = new Pose(49, 115, Math.toRadians(160));
    private final Pose park = new Pose(41, 84, Math.toRadians(134));

    private PathChain shoot1, collect1, shoot2, GateCollect1, shoot3, GateCollect2, shoot4,
            GateCollect3, shoot5, collect3, shoot6;

    @Override
    public void init() {
        pathState = PathState.start;
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0, 0.362);
        blocker.setPosition(0);
        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0, 0.0761);
        hood.setPosition(1);
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = ConstantsNewBot.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        collect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, goToCollect1ControlPoint, collect1thing))
                .setTangentHeadingInterpolation()
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(collect1thing, shoot2ControlPoint1, shootPose2))
                .setLinearHeadingInterpolation(collect1thing.getHeading(), shootPose2.getHeading())
                .build();

        GateCollect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, shootPose2ToGateControlPoint, gateCollect1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gateCollect1.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect1, shootPose2ToGateControlPoint, shootBall3))
                .setLinearHeadingInterpolation(gateCollect1.getHeading(), shootBall3.getHeading())
                .build();

        GateCollect2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall3, shootPose2ToGateControlPoint, gateCollect2))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), gateCollect2.getHeading())
                .build();

        shoot4 = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect2, shootPose2ToGateControlPoint, shootBall4))
                .setLinearHeadingInterpolation(gateCollect2.getHeading(), shootBall4.getHeading())
                .build();

        GateCollect3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall4, shootPose2ToGateControlPoint, gateCollect3))
                .setLinearHeadingInterpolation(shootBall4.getHeading(), gateCollect3.getHeading())
                .build();

        shoot5 = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect3, shootPose2ToGateControlPoint, shootBall5))
                .setLinearHeadingInterpolation(gateCollect3.getHeading(), shootBall5.getHeading())
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall5, shoot4ToCollect3ControlPoint, collect3end))
                .setLinearHeadingInterpolation(shootBall5.getHeading(), collect3end.getHeading())
                .build();

        shoot6 = follower.pathBuilder()
                .addPath(new BezierLine(collect3end, shootBall6))
                .setLinearHeadingInterpolation(collect3end.getHeading(), shootBall6.getHeading())
                .build();
    }

    /** Distance from current pose to blue goal (inches). */
    private double getDistanceToGoal() {
        Pose p = follower.getPose();
        double dx = BLUE_GOAL_X - p.getX();
        double dy = BLUE_GOAL_Y - p.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Lead target for shooting while moving: goal minus velocity * shotTime (same idea as teleop sillyTarget). */
    private Pose getSillyTargetPose() {
        double dist = getDistanceToGoal();
        double t = ShotTimeLookupTable.getTime(dist);
        Vector v = follower.getVelocity();
        if (v == null || (dist >= 130)) {
            return new Pose(BLUE_GOAL_X, BLUE_GOAL_Y, 0);
        }
        double vx = v.getMagnitude() * Math.cos(v.getTheta());
        double vy = v.getMagnitude() * Math.sin(v.getTheta());
        return new Pose(
                BLUE_GOAL_X - vx * t,
                BLUE_GOAL_Y - vy * t,
                0
        );
    }

    /** Turret angle (deg) to aim at a target pose from current pose (for rotator RUN_TO_POSITION). */
    private double getTurretAngleDegForPose(Pose target) {
        Pose robot = follower.getPose();
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();
        double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg = Math.toDegrees(robot.getHeading());
        double turretDeg = angleToTargetDeg - headingDeg;
        while (turretDeg > 180) turretDeg -= 360;
        while (turretDeg < -180) turretDeg += 360;
        return -turretDeg;
    }

    /** True when robot is approaching the shoot line (0,144)-(72,72) and is within LEAD_BEFORE_SHOOT_LINE before crossing (x+y < 144, 144-x-y in (0, LEAD]). */
    private boolean isApproachingShootLineBeforeCrossing() {
        Pose p = follower.getPose();
        double sum = p.getX() + p.getY();
        double gap = 144.0 - sum;
        return gap > 0 && gap <= LEAD_BEFORE_SHOOT_LINE;
    }

    /** Update rotator aim at silly target and launcher velocity from distance. Call every loop during move-shot states. */
    private void updateMoveShotAimAndVelocity() {
        Pose silly = getSillyTargetPose();
        double angleDeg = getTurretAngleDegForPose(silly);
        int targetTicks = rotatorStartPosition + (int) ((angleDeg / 180.0) * motor180Range);
        rotator.setTargetPosition(targetTicks);
        double dist = getDistanceToGoal();
        launcher.setVelocity(VelocityLookupTable.getVelocity(dist));
        hood.setPosition(1);
    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                theWheelOfTheOx.setPower(0);
                launcher.setVelocity(1160);
                tree.setPower(1);
                blocker.setPosition(0);
                launcher.setVelocity(1160);
                hood.setPosition(1);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(shoot1);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    theWheelOfTheOx.setPower(-1);
                }
                rotator.setTargetPosition(rotatorStartPosition);
                setPathState(PathState.actuallyshoot1);
                break;

            case actuallyshoot1:
                rotator.setTargetPosition(rotatorStartPosition);
                launcher.setVelocity(1160);
                if (pathTimer.getElapsedTimeSeconds() > 1.9) {
                    blocker.setPosition(1);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.2) {
                    tree.setPower(1);
                    launcher.setVelocity(1160);
                    hood.setPosition(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    theWheelOfTheOx.setPower(-1);
                    if (pathTimer.getElapsedTimeSeconds() > 2.7) {
                        setPathState(PathState.collection);
                    }
                }
                break;

            case collection:
                rotator.setTargetPosition(rotatorStartPosition);
                hood.setPosition(1);
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(0);
                tree.setPower(1);
                if (!follower.isBusy() && !collectionStarted) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    launcher.setVelocity(1160);
                    hood.setPosition(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    follower.followPath(collect1);
                    collectionStarted = true;
                    if (pathTimer.getElapsedTimeSeconds() > 1.25) {
                        theWheelOfTheOx.setPower(-1);
                    }
                }
                if (!follower.isBusy() && collectionStarted) {
                    setPathState(PathState.shoot);
                }
                break;

            case shoot:
                if (!follower.isBusy() && !shoot2Started) {
                    hood.setPosition(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot2);
                    launcher.setVelocity(1160);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot2Started = true;
                }
                if (shoot2Started) {
                    updateMoveShotAimAndVelocity();
                    if (!moveShotBlockerOpened && isApproachingShootLineBeforeCrossing()) {
                        blocker.setPosition(1);
                        moveShotBlockerOpened = true;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                }
                if (!follower.isBusy() && shoot2Started && pathTimer.getElapsedTimeSeconds() > MOVE_SHOT_DWELL_SEC) {
                    setPathState(PathState.GateCollection);
                }
                break;

            case GateCollection:
                rotator.setTargetPosition(rotatorStartPosition);
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(0);
                if (!follower.isBusy() && !gateCollectionStarted) {
                    follower.followPath(GateCollect1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    launcher.setVelocity(1160);
                    tree.setPower(1);
                    hood.setPosition(1);
                    gateCollectionStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.25) {
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && gateCollectionStarted && pathTimer.getElapsedTimeSeconds() > 3.15) {
                    setPathState(PathState.shootAgain);
                }
                break;

            case shootAgain:
                if (!follower.isBusy() && !shoot3Started) {
                    hood.setPosition(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot3);
                    tree.setPower(1);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    shoot3Started = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    tree.setPower(0);
                }
                if (shoot3Started) {
                    updateMoveShotAimAndVelocity();
                    if (!moveShotBlockerOpened && isApproachingShootLineBeforeCrossing()) {
                        blocker.setPosition(1);
                        moveShotBlockerOpened = true;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.55) {
                        tree.setPower(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.675) {
                        theWheelOfTheOx.setPower(-1);
                    }
                }
                if (!follower.isBusy() && shoot3Started && pathTimer.getElapsedTimeSeconds() > 2.75) {
                    setPathState(PathState.GateCollectionAgain);
                }
                break;

            case GateCollectionAgain:
                if (!follower.isBusy() && !gateCollectionAgainStarted) {
                    blocker.setPosition(0);
                    theWheelOfTheOx.setPower(0);
                    follower.followPath(GateCollect2);
                    hood.setPosition(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    launcher.setVelocity(1160);
                    tree.setPower(1);
                    gateCollectionAgainStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.25) {
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && gateCollectionAgainStarted && pathTimer.getElapsedTimeSeconds() > 3.15) {
                    setPathState(PathState.shootAgainAgain);
                }
                break;

            case shootAgainAgain:
                tree.setPower(1);
                rotator.setTargetPosition(rotatorStartPosition);
                if (!follower.isBusy() && !shoot4Started) {
                    hood.setPosition(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot4);
                    tree.setPower(1);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    shoot4Started = true;
                }
                if (shoot4Started) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.9) {
                        tree.setPower(0);
                    }
                    updateMoveShotAimAndVelocity();
                    if (!moveShotBlockerOpened && isApproachingShootLineBeforeCrossing()) {
                        blocker.setPosition(1);
                        moveShotBlockerOpened = true;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.35) {
                        tree.setPower(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                        theWheelOfTheOx.setPower(-1);
                    }
                }
                if (!follower.isBusy() && shoot4Started && pathTimer.getElapsedTimeSeconds() > 2.8) {
                    setPathState(PathState.GateCollectionAgainAgain);
                }
                break;

            case GateCollectionAgainAgain:
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(0);
                if (!follower.isBusy() && !gateCollectionAgainStarted) {
                    follower.followPath(GateCollect3);
                    rotator.setTargetPosition(rotatorStartPosition);
                    launcher.setVelocity(1160);
                    tree.setPower(1);
                    hood.setPosition(1);
                    gateCollectionAgainStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds() > 2.25) {
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && gateCollectionAgainStarted && pathTimer.getElapsedTimeSeconds() > 3.55) {
                    setPathState(PathState.shootAgainAgainAgain);
                }
                break;

            case shootAgainAgainAgain:
                tree.setPower(1);
                rotator.setTargetPosition(rotatorStartPosition);
                if (!follower.isBusy() && !shoot4Started) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot5);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot4Started = true;
                }
                if (shoot4Started) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.9) {
                        tree.setPower(0);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.35) {
                        tree.setPower(1);
                    }
                    updateMoveShotAimAndVelocity();
                    if (!moveShotBlockerOpened && isApproachingShootLineBeforeCrossing()) {
                        blocker.setPosition(1);
                        moveShotBlockerOpened = true;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        theWheelOfTheOx.setPower(-1);
                    }
                }
                if (!follower.isBusy() && shoot4Started && pathTimer.getElapsedTimeSeconds() > 2.55) {
                    setPathState(PathState.collectAgainAgainEnd);
                }
                break;

            case collectAgainAgainEnd:
                rotator.setTargetPosition(rotatorStartPosition);
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(0);
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && !collectionStarted) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    launcher.setVelocity(1160);
                    hood.setPosition(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    follower.followPath(collect3);
                    collectionStarted = true;
                }
                if (!follower.isBusy() && collectionStarted && pathTimer.getElapsedTimeSeconds() > 0.25) {
                    theWheelOfTheOx.setPower(-1);
                    setPathState(PathState.shootAgainAgainAgainAgain);
                }
                break;

            case shootAgainAgainAgainAgain:
                rotator.setTargetPosition(rotatorStartPosition);
                if (!follower.isBusy() && !shoot5Started) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot6);
                    launcher.setVelocity(1160);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot5Started = true;
                }
                if (!follower.isBusy() && shoot5Started) {
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        blocker.setPosition(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.75) {
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.75) {
                        setPathState(PathState.done);
                    }
                }
                break;

            case done:
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        moveShotBlockerOpened = false;
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
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        tree = hardwareMap.get(DcMotor.class, "tree");
        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0, 0.0761);
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0, 0.4);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorStartPosition = 0;
        rotator.setTargetPosition(rotatorStartPosition);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double P = 400, I = 0, D = 0, F = 13.2965;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        PoseStorage.savePose(follower.getPose());

        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("jolly crusader velocity", launcher.getVelocity());
        telemetry.addData("near shoot line", isApproachingShootLineBeforeCrossing());
        telemetry.addData("moveShotBlockerOpened", moveShotBlockerOpened);
    }
}
