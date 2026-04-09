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
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.robotControl.RedUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.LookUpTables.ShotTimeLookupTable;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.LookUpTables.VelocityLookupTable;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer.TransferGate;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;
import org.firstinspires.ftc.teamcode.tests.ColorTesting;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.security.cert.PKIXRevocationChecker;

@Autonomous(name = "thingy", group = "auto", preselectTeleOp = "AmazingBotRed")
public class thingy extends OpMode {
    private static final int PIPELINENUM = 0;
    Pose target = new Pose(144, 144, Math.toRadians(36));
    Pose sillyTarget;
    private int rotatorStartPosition=0;
    double txDeg = 0.0;
    double tyDeg = 0.0;
    private Follower follower;
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

    private final Pose startPose = new Pose(87.3, 7.5, Math.toRadians(90));

    private final Pose shootPose1 = new Pose(132.5, 60.25, Math.toRadians(38));
    //private final Pose shootPose1 = new Pose(133, 61, Math.toRadians(40)); TODO works kinda, watch IMG_0754
    private final Pose collect1thing = new Pose(126, 58, Math.toRadians(0));
    private final Pose goToCollect1ControlPoint = new Pose(91.735, 60.091);
    private final Pose shootPose2 = new Pose(82, 77, Math.toRadians(25));
    private final Pose gateCollect1 = new Pose(129, 66, Math.toRadians(45));
    private final Pose option1 = new Pose(131, 61, Math.toRadians(45));
    private final Pose option2 = new Pose(137, 55, Math.toRadians(90));

    private final Pose shootBall3 = new Pose(82, 77, Math.toRadians(25));
    private final Pose inBetween2 = new Pose(100, 62, Math.toRadians(202.5));
    private final Pose gateCollect2 = new Pose(128.5, 62, Math.toRadians(205));
    private final Pose shootBall4 = new Pose(84, 75, Math.toRadians(230));
    private final Pose gateCollect3 = new Pose(128.5, 62, Math.toRadians(205));
    private final Pose shootBall5 = new Pose(95, 87, Math.toRadians(230));
    private final Pose collect3end = new Pose(116, 85, Math.toRadians(0));
    private final Pose collect3ControlPoint = new Pose(102.98, 84.857);
    private final Pose shootBall6 = new Pose(89, 120, Math.toRadians(33));
    private final Pose park = new Pose(103, 84, Math.toRadians(226));

    private PathChain shoot1, Turn, goToCollect1, shoot3ToGate, collect1, shoot2, GateCollect3, shoot6, InBetween1, InBetween2, GateCollect1, GateCollect2, shoot3, awayfromGate, goToCollect3, collect3, shoot4, goToGate, openGate, goToCollect4, collect4, shoot5, parking;

    public void buildPaths() {
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
                .addPath(new BezierLine(shootPose2, option1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), option1.getHeading())
                .build();
        Turn = follower.pathBuilder()
                .addPath(new BezierLine(gateCollect1, option1))
                .setLinearHeadingInterpolation(gateCollect1.getHeading(), option1.getHeading())
                .build();


        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(option1, shootBall3))
                .setLinearHeadingInterpolation(option1.getHeading(), shootBall3.getHeading())
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
        switch (pathState) {
            case start:
                if (!shoot1Started) {
                    follower.followPath(shoot1);
                    robot.intake.shift();
                    robot.gate.block();
                    robot.intake.powerON();
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    shoot1Started = true;
                }
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
    }

    @Override
    public void init() {
        pathState = PathState.start;

        pathTimer = new Timer();
        opModeTimer = new Timer();
        gateTimer = new Timer();
        shootTimer = new Timer();
        openTimer = new Timer();

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
        robot.intake.down();
        robot.gate.block();

        sillyTarget = robot.chassisLocal.sillyTargetPose(target);

        telemetry.addLine("Good to go RED");
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

        Vector velocity = follower.getVelocity();
        double vx = velocity.getMagnitude() * Math.cos(velocity.getTheta());
        double vy = velocity.getMagnitude() * Math.sin(velocity.getTheta());
        double shotTime = ShotTimeLookupTable.getTime(dist);
        Pose aimTarget;
        if (dist < 120) {
            aimTarget = new Pose(
                    target.getX() - vx * shotTime,
                    target.getY() - vy * shotTime
            );
        } else {
            aimTarget = target;
        }

        double aimDx = aimTarget.getX() - currentPose.getX();
        double aimDy = aimTarget.getY() - currentPose.getY();
        double angleToGoal = Math.toDegrees(Math.atan2(aimDy, aimDx));
        double turretAngle = angleToGoal - Math.toDegrees(currentPose.getHeading());
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;
        robot.turret.setRotatorToAngle(-turretAngle);

        robot.turret.setVelocity(VelocityLookupTable.getVelocity(dist));
        int zone = VelocityLookupTable.getZone(dist);
        if (zone == 1) robot.turret.setHoodPos(RedUniversalConstants.CLOSE_HOOD_POSITION);
        else if (zone == 2) robot.turret.setHoodPos(RedUniversalConstants.MID_HOOD_POSITION);
        else robot.turret.setHoodPos(RedUniversalConstants.FAR_HOOD_POSITION);

        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", currentPose.getX());
        telemetry.addData("y", currentPose.getY());
        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Dist to target", dist);
        telemetry.addData("Turret Angle", -turretAngle);
        telemetry.addData("Launcher velocity", robot.turret.getTargetVelocity());
        telemetry.addData("LauncherL velocity", robot.turret.getVelocityL());
        telemetry.addData("LauncherR velocity", robot.turret.getVelocityR());
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
}
