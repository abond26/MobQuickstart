package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.geometry.BezierLine;
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
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TurretConstants;

@Autonomous(name = "sideways 15 far red", group = "sideways")
public class red15farside extends OpMode {
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;

    // Flags to prevent path oscillation - ensure paths are only called once per state
    private boolean shoot2Started = false;
    private boolean shoot3Started = false;
    private boolean shoot4Started = false;
    private boolean shoot5Started = false;
    private boolean collectAgainStarted = false;
    private boolean collectAgainEndStarted = false;
    private boolean collectAgainAgainStarted = false;
    private boolean collectAgainAgainEndStarted = false;
    private boolean collectAgainAgainStartStarted = false;
    private boolean collectAgainAgainAgainStarted = false;
    private boolean collectAgainAgainAgainEndStarted = false;
    private boolean parkingStarted = false;

    private Servo hood, blocker;
    private int limeHeight = 33;
    private int offset = 28;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 1; // tune this

    // Hood adjustment constants (from TesterinoBlue)
    private static final double DISTANCE_THRESHOLD = 180.0;
    private static final double CLOSE_HOOD_POSITION = .2541; // Hood position for close shots
    private static final double FAR_HOOD_POSITION = 0.36; // Hood position for far shots

    private int y = tagHeight - limeHeight;

    // Localization: pose comes from dead wheels only (Pinpoint localizer in ConstantsNewBot.createFollower), same as SubsysTele.
    /** Red backboard / goal target for localization-based rotator aiming (field coords). */
    private static final double RED_GOAL_X = 144;
    private static final double RED_GOAL_Y = 144;
    /** Degrees added to turret angle to correct aim (positive = aim more right). Tune if shots go left/right. */
    private static final double RED_AIM_OFFSET_DEG = 2.5;
    /** Rotator must be within this many ticks of goal angle before we open blocker / feed. */
    private static final int ROTATOR_AIM_TOLERANCE_TICKS = 35;
    /** Only aim rotator when chassis speed is below this (in/s) so we lock on when robot is at the point and not moving. */
    private static final double CHASSIS_AT_REST_THRESHOLD = 2.0;

    //Rotator: use Turret subsystem (same as SubsysTele – direction, power, rotator180Range)
    int limelightMotor180Range = 910;
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
    private DcMotor tree, theWheelOfTheOx;
    private Turret turret;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        shooting1,
        shooting2,
        shooting3,
        shooting4,
        start,
        actuallyshoot1,
        gotocollect,
        collection,
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
        collection4,

    }


    PathState pathState;
    // Mirrored from blue: redX = 144 - blueX, redHeading = Math.PI - blueHeading
    private final Pose startPose = new Pose(88.3, 8.5, Math.toRadians(0));
    private final Pose shootPose1 = new Pose(89, 15, Math.toRadians(67));
    private final Pose collect1ControlPoint = new Pose(96.19073569482288, 35.68937329700272);

    private final Pose collect1thing = new Pose(131, 36, Math.toRadians(0));
    private final Pose shootPose2 = new Pose(89, 15, Math.toRadians(66));

    private final Pose collect2End = new Pose(132, 12, Math.toRadians(0));
    private final Pose collect2End2 = new Pose(132, 10, Math.toRadians(0));

    private final Pose shootBall3 = new Pose(89, 15, Math.toRadians(69));
    private final Pose collect3Start = new Pose(129, 10, Math.toRadians(0));
    private final Pose shootBall4 = new Pose(89, 15, Math.toRadians(69));
    private final Pose collect4Start = new Pose(129, 10, Math.toRadians(0));
    private final Pose shootBall5 = new Pose(89, 15, Math.toRadians(69));
    private final Pose park = new Pose(103, 22, Math.toRadians(0));
    private PathChain shoot1, collect5, goToCollect1, collect2StartAgain, collect2EndAgain, collect1, shoot2, goToCollect2, collect2, shoot3,goToCollect2Again, collect2Again, goToCollect2AgainAgain, collect2AgainAgain, goToCollect3, collect3, shoot4, goToCollect4, collect4, shoot5, parking;

    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        collect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, collect1ControlPoint, collect1thing))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), collect1thing.getHeading())
                .build();

        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1thing, shootPose2))
                .setLinearHeadingInterpolation(collect1thing.getHeading(), shootPose2.getHeading())
                .build();

        collect2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, collect2End))
                .setTangentHeadingInterpolation()
                .build();
        collect2Again = follower.pathBuilder()
                .addPath(new BezierLine(collect2End, collect2End2))
                .setLinearHeadingInterpolation(collect2End.getHeading(), collect2End2.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(collect2End2, shootBall3))
                .setLinearHeadingInterpolation(collect2End2.getHeading(), shootBall3.getHeading())
                .build();
        collect4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall3, collect3Start))
                .setTangentHeadingInterpolation()
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(collect3Start, shootBall4))
                .setLinearHeadingInterpolation(collect3Start.getHeading(), shootBall4.getHeading())
                .build();
        collect5 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall4, collect4Start))
                .setTangentHeadingInterpolation()
                .build();
        shoot5 = follower.pathBuilder()
                .addPath(new BezierLine(collect4Start, shootBall5))
                .setLinearHeadingInterpolation(collect4Start.getHeading(), shootBall5.getHeading())
                .build();

        parking = follower.pathBuilder()
                .addPath(new BezierLine(shootBall5, park))
                .setLinearHeadingInterpolation(shootBall5.getHeading(), park.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                hood.setPosition(0);
                blocker.setPosition(0);
                launcher.setVelocity(1540);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(shoot1);
                turret.setRotatorPos(0);
                setPathState(PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                launcher.setVelocity(1540);
                if (!follower.isBusy()) {
                    if (isRobotAtRest() && !isRotatorAimedAtGoal()) aimRotatorAtRedGoal();
                    if (launcher.getVelocity() >= 1520 && launcher.getVelocity() <= 1560 && isRotatorAimedAtGoal()) {
                        blocker.setPosition(1);
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                } else {
                    turret.setRotatorPos(0);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState((PathState.collection));
                }
                break;

            case collection:
                blocker.setPosition(0);
                tree.setPower(1);
                theWheelOfTheOx.setPower(-0.5);
                follower.followPath(collect1);
                launcher.setVelocity(1540);
                turret.setRotatorPos(0);
                follower.setMaxPower(INTAKE_DRIVE_POWER);

                setPathState((red15farside.PathState.shoot));

                break;
            case shoot:
                tree.setPower(1);
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(-0.25);
                turret.setRotatorPos(0);
                if (!follower.isBusy() && !shoot2Started) {
                    launcher.setVelocity(1540);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    follower.followPath(shoot2);
                    shoot2Started = true;
                    theWheelOfTheOx.setPower(-1);
                    setPathState((red15farside.PathState.shooting1));
                }
                break;
            case shooting1:
                launcher.setVelocity(1540);
                if (!follower.isBusy()) {
                    if (isRobotAtRest() && !isRotatorAimedAtGoal()) aimRotatorAtRedGoal();
                    if (launcher.getVelocity() >= 1520 && launcher.getVelocity() <= 1560 && isRotatorAimedAtGoal()) {
                        blocker.setPosition(1);
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                } else {
                    turret.setRotatorPos(0);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4.5) {
                    setPathState((PathState.collectAgainEnd));
                }
                break;
            case collectAgainEnd:
                tree.setPower(1);
                theWheelOfTheOx.setPower(-0.25);
                blocker.setPosition(0);
                turret.setRotatorPos(0);
                if (!follower.isBusy() && !collectAgainEndStarted) {
                    follower.followPath(collect2);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(0);
                    turret.setRotatorPos(0);
                    tree.setPower(1);
                    collectAgainEndStarted = true;
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && collectAgainEndStarted) {
                    setPathState((PathState.collectAgainAgainEnd));
                }
                break;
            case collectAgainAgainEnd:
                theWheelOfTheOx.setPower(-0.25);
                tree.setPower(1);
                turret.setRotatorPos(0);
                blocker.setPosition(0);
                if (!follower.isBusy() && !collectAgainAgainEndStarted) {
                    follower.followPath(collect2Again);
                    turret.setRotatorPos(0);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(0);
                    tree.setPower(1);
                    collectAgainAgainEndStarted = true;
                }
                if (!follower.isBusy() && collectAgainAgainEndStarted) {
                    setPathState((PathState.shootAgain));
                }
                break;
            case shootAgain:
                tree.setPower(1);
                theWheelOfTheOx.setPower(-0.25);
                launcher.setVelocity(1540);
                turret.setRotatorPos(0);
                if (!follower.isBusy() && !shoot3Started) {
                    launcher.setVelocity(1540);
                    follower.followPath(shoot3);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    theWheelOfTheOx.setPower(-1);
                    shoot3Started = true;
                    setPathState((PathState.shooting2));
                }
                break;
            case shooting2:
                launcher.setVelocity(1540);
                if (!follower.isBusy()) {
                    if (isRobotAtRest() && !isRotatorAimedAtGoal()) aimRotatorAtRedGoal();
                    if (launcher.getVelocity() >= 1520 && launcher.getVelocity() <= 1560 && isRotatorAimedAtGoal()) {
                        blocker.setPosition(1);
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                } else {
                    turret.setRotatorPos(0);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5) {
                    setPathState((PathState.collectAgainAgainAgain));
                }
                break;
            case collectAgainAgainAgain:
                tree.setPower(1);
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(-0.25);
                turret.setRotatorPos(0);
                if (!follower.isBusy() && !collectAgainAgainAgainStarted) {
                    follower.followPath(collect4);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(-0.25);
                    turret.setRotatorPos(0);
                    tree.setPower(1);
                    collectAgainAgainAgainStarted = true;
                }
                if (!follower.isBusy() && collectAgainAgainAgainStarted) {
                    setPathState((PathState.shootAgainAgain));
                }
                break;
            case shootAgainAgain:
                tree.setPower(1);
                theWheelOfTheOx.setPower(-0.25);
                launcher.setVelocity(1540);
                turret.setRotatorPos(0);
                if (!follower.isBusy() && !shoot3Started) {
                    launcher.setVelocity(1540);
                    follower.followPath(shoot4);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot3Started = true;
                    setPathState((PathState.shooting3));
                }
                break;
            case shooting3:
                launcher.setVelocity(1540);
                if (!follower.isBusy()) {
                    if (isRobotAtRest() && !isRotatorAimedAtGoal()) aimRotatorAtRedGoal();
                    if (launcher.getVelocity() >= 1520 && launcher.getVelocity() <= 1560 && isRotatorAimedAtGoal()) {
                        blocker.setPosition(1);
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                } else {
                    turret.setRotatorPos(0);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState((PathState.collection4));
                }
                break;
            case collection4:
                tree.setPower(1);
                blocker.setPosition(0);
                turret.setRotatorPos(0);
                if (!follower.isBusy() && !collectAgainAgainAgainEndStarted) {
                    follower.followPath(collect5);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(-1);
                    turret.setRotatorPos(0);
                    tree.setPower(1);
                    collectAgainAgainAgainEndStarted = true;
                }
                if (!follower.isBusy() && collectAgainAgainAgainEndStarted) {
                    theWheelOfTheOx.setPower(0);
                    setPathState((PathState.shootAgainAgainAgain));
                }
                break;
            case shootAgainAgainAgain:
                launcher.setVelocity(1540);
                turret.setRotatorPos(0);
                if (!follower.isBusy() && !shoot5Started) {
                    launcher.setVelocity(1540);
                    follower.followPath(shoot5);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(0.6);
                    shoot5Started = true;
                    setPathState((PathState.shooting4));

                }
                break;
            case shooting4:
                launcher.setVelocity(1540);
                if (!follower.isBusy()) {
                    if (isRobotAtRest() && !isRotatorAimedAtGoal()) aimRotatorAtRedGoal();
                    if (launcher.getVelocity() >= 1520 && launcher.getVelocity() <= 1560 && isRotatorAimedAtGoal()) {
                        blocker.setPosition(1);
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                } else {
                    turret.setRotatorPos(0);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState((PathState.parklol));
                }
                break;


            case parklol:
                turret.setRotatorPos(0);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1 && !parkingStarted) {
                    theWheelOfTheOx.setPower(-1);
                    follower.followPath(parking);
                    parkingStarted = true;
                }
                if (!follower.isBusy() && parkingStarted && pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState((PathState.done));
                }
                break;
            case done:
                turret.setRotatorPos(0);
                break;

        }
    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shoot2Started = false;
        shoot3Started = false;
        shoot4Started = false;
        shoot5Started = false;
        collectAgainStarted = false;
        collectAgainEndStarted = false;
        collectAgainAgainStarted = false;
        collectAgainAgainEndStarted = false;
        collectAgainAgainStartStarted = false;
        collectAgainAgainAgainStarted = false;
        collectAgainAgainAgainEndStarted = false;
        parkingStarted = false;
    }

    @Override
    public void init() {
        pathState = PathState.start;
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0, 0.362);
        blocker.setPosition(0);
        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0,0.0761);
        hood.setPosition(0);
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = ConstantsNewBot.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

        turret = new Turret(hardwareMap);

        tree = hardwareMap.get(DcMotor.class, "tree");
        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double P = 400;
        double I = 0;
        double D = 0;
        double F = 13.2965;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        follower.update(); // dead-wheel localization + path following (same as SubsysTele chassisLocal.update())
        statePathUpdate();
        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("jolly crusader velocity", launcher.getVelocity());
        if (turret != null) {
            telemetry.addData("rotator pos", turret.getRotatorPos());
            telemetry.addData("rotator target ticks", getRotatorTargetTicksForRedGoal());
            telemetry.addData("rotator aimed", isRotatorAimedAtGoal());
        }
    }

    public double getDist(double tyDeg) {
        double tyRad = Math.abs(Math.toRadians(tyDeg + limelightUpAngle));
        double dist = y / Math.tan(tyRad);
        double realDist = 0.55 * dist + 40.3;
        return realDist;
    }
    public double calcVelocity(double dist) {
        double velocity = 3.30933 * dist + 1507.01002;
        return velocity;
    }

    public void intake(double intakePower){
        tree.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-0.3);
        }
    }
    /**
     * Turret angle (deg) to aim at red goal.
     * Same flow as SubsysTele blue goal: RobotActions.aimRotatorLocal(target) uses
     * ChassisLocal.calculateTurretAngle(target) then Turret.setRotatorToAngle(angle).
     * Here: pose from follower (dead-wheel localization, same as chassisLocal.update()),
     * same math as ChassisLocal.calculateTurretAngle (angleToGoal - heading, normalized, return -turretAngle).
     */
    private double getTurretAngleDegForRedGoal() {
        Pose robot = follower.getPose();
        double dx = RED_GOAL_X - robot.getX();
        double dy = RED_GOAL_Y - robot.getY();
        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg = Math.toDegrees(robot.getHeading());
        double turretAngleDeg = angleToGoalDeg - headingDeg;
        while (turretAngleDeg > 180) turretAngleDeg -= 360;
        while (turretAngleDeg < -180) turretAngleDeg += 360;
        return -turretAngleDeg + RED_AIM_OFFSET_DEG;
    }

    /** Target ticks for red goal (Turret subsystem uses TurretConstants.rotator180Range). */
    private int getRotatorTargetTicksForRedGoal() {
        double fracOf180 = Math.toRadians(getTurretAngleDegForRedGoal()) / Math.PI;
        return TurretConstants.ROTATOR_ZERO_TICKS + (int) (fracOf180 * TurretConstants.rotator180Range);
    }

    /** Aims the rotator at the red goal: same as SubsysTele aimRotatorLocal(blue target) but for red goal. */
    private void aimRotatorAtRedGoal() {
        if (turret == null) return;
        turret.setRotatorToAngle(getTurretAngleDegForRedGoal());
    }

    /** True if the rotator is aimed at the red goal within tolerance (so safe to open blocker and feed). */
    private boolean isRotatorAimedAtGoal() {
        if (turret == null) return false;
        int targetTicks = getRotatorTargetTicksForRedGoal();
        int currentTicks = turret.getRotatorPos();
        return Math.abs(currentTicks - targetTicks) <= ROTATOR_AIM_TOLERANCE_TICKS;
    }

    /** True when chassis speed is below threshold – robot at the point and not moving so we can lock on. */
    private boolean isRobotAtRest() {
        Vector v = follower.getVelocity();
        return v != null && v.getMagnitude() < CHASSIS_AT_REST_THRESHOLD;
    }

    /** Red side: rotator adjustment uses - offset (mirrored from blue's + offset). Uses Turret for position. */
    public void adjustRotator(double tx) {
        if (turret == null) return;
        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * limelightMotor180Range);
        int newPosition = turret.getRotatorPos() + adjustment - offset;
        turret.setRotatorPos(newPosition);
    }

    public void adjustHoodBasedOnDistance(double distance) {
        if (hood != null) {
            if (distance > DISTANCE_THRESHOLD) {
                hood.setPosition(FAR_HOOD_POSITION);
            } else {
                hood.setPosition(CLOSE_HOOD_POSITION);
            }
        }
    }

    public void updateLimelightAdjustments() {
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            if (ll != null && ll.isValid()) {
                txDeg = ll.getTx();
                tyDeg = ll.getTy();

                lastValidTx = txDeg;
                lastValidTy = tyDeg;

                double currentDistance = getDist(tyDeg);
                if (currentDistance > 0) {
                    lastValidDistance = currentDistance;
                    hasValidLimelightData = true;

                    adjustRotator(txDeg);

                    launcher.setVelocity(calcVelocity(currentDistance));
                    adjustHoodBasedOnDistance(currentDistance);
                }
            } else {
                if (hasValidLimelightData) {
                    adjustRotator(lastValidTx);
                    if (lastValidDistance > 0) {
                        launcher.setVelocity(calcVelocity(lastValidDistance));
                        adjustHoodBasedOnDistance(lastValidDistance);
                    }
                }
            }
        }
    }

    public void resetLimelightData() {
        hasValidLimelightData = false;
        lastValidTx = 0.0;
        lastValidTy = 0.0;
        lastValidDistance = 0.0;
    }

}
