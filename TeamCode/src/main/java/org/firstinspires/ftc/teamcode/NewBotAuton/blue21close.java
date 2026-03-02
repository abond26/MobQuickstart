package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "tangential Blue 21 close ", group = "new bot")
public class blue21close extends OpMode {
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
    // Extra flags for the additional gate cycle
    private boolean gateCollectionAgainAgainExtraStarted = false;
    private boolean shoot6Started = false;
    private boolean shoot7Started = false;

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

    /** Blue goal for localization-based rotator aiming (field coords). */
    private static final double BLUE_GOAL_X = 1;
    private static final double BLUE_GOAL_Y = 144;
    private static final double BLUE_AIM_OFFSET_DEG = 0.5;
    /** Only aim rotator when chassis speed is below this (in/s). */
    private static final double CHASSIS_AT_REST_THRESHOLD = 2.0;

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
    private DcMotor tree, theWheelOfTheOx;
    private Turret turret;
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
        // Extra cycle: another gate collection + shot
        GateCollectionAgainAgainExtra,
        shootAgainAgainAgainExtra,
        shootAgainAgainAgainAgain,

    }

    PathState pathState;
    // Mirrored coordinates: blueX = 144 - redX, blueHeading = Math.PI - redHeading
    private final Pose startPose = new Pose(26.7, 132, Math.toRadians(144));
    private final Pose shootPose1 = new Pose(46, 97.5, Math.toRadians(270));
    //private final Pose collect1thingstart = new Pose(56, 59, Math.toRadians(180));
    private final Pose collect1thing = new Pose(19, 61, Math.toRadians(180));
    private final Pose goToCollect1ControlPoint = new Pose(65, 58.5, Math.toRadians(180));
    private final Pose shootPose2 = new Pose( 46, 97.5, Math.toRadians(270));
    
    // Control points for shoot2 path
    private final Pose shoot2ControlPoint1 = new Pose(49.15667574931882, 76, Math.toRadians(180));
    private final Pose gateCollect1 = new Pose( 15, 62, Math.toRadians(147));
    //private final Pose inBetween1 = new Pose(44, 62, Math.toRadians(157.5));
    private final Pose shootPose2ToGateControlPoint = new Pose(20.94414168937329, 58.42302452316075, Math.toRadians(180));
    private final Pose shootBall3 = new Pose(56, 82.5, Math.toRadians(270));
    private final Pose inBetween2 = new Pose(44, 62, Math.toRadians(157.5));
    private final Pose gateCollect2 = new Pose( 15, 62, Math.toRadians(147));
    private final Pose shootBall4 = new Pose(56, 82.5, Math.toRadians(270));
    private final Pose gateCollect3 = new Pose( 15, 62, Math.toRadians(147));
    private final Pose shootBall5 = new Pose(56, 82.5, Math.toRadians(270));
    private final Pose gateCollect4 = new Pose( 15, 62, Math.toRadians(147));
    private final Pose shootBall7 = new Pose(56, 82.5, Math.toRadians(270));

    //private final Pose collect3start=new Pose(57, 86, Math.toRadians(180));
    //private final Pose shoot4ToCollect3ControlPoint = new Pose(41.25340599455039, 82.36784741144412, Math.toRadians(180));

    //
    private final Pose collect3end = new Pose(24, 86, Math.toRadians(180));
    private final Pose shootBall6 = new Pose(49, 115, Math.toRadians(180));

    private final Pose park = new Pose(41, 84, Math.toRadians(134));



    private PathChain shoot1, goToCollect1, GateCollect4, shoot7,collect1, shoot2, GateCollect3, shoot6, InBetween1, InBetween2, GateCollect1, GateCollect2, shoot3, awayfromGate, goToCollect3, collect3, shoot4, goToGate, openGate, goToCollect4, collect4, shoot5, parking;

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
                .addPath(new BezierCurve(collect1thing, shoot2ControlPoint1, shootPose2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        GateCollect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, shootPose2ToGateControlPoint, gateCollect1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gateCollect1.getHeading())
                .build();

//        GateCollect1 = follower.pathBuilder()
//                .addPath(new BezierLine(gateCollect1, gateCollect1))
//                .setLinearHeadingInterpolation(gateCollect1.getHeading(), gateCollect1.getHeading())
//                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect1, shootPose2ToGateControlPoint, shootBall3))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        GateCollect2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall3, shootPose2ToGateControlPoint, gateCollect2))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), gateCollect2.getHeading())
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
                .setLinearHeadingInterpolation(shootBall4.getHeading(), gateCollect3.getHeading())
                .build();
        shoot5 = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect3, shootPose2ToGateControlPoint, shootBall5))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        GateCollect4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootBall5, shootPose2ToGateControlPoint, gateCollect4))
                .setLinearHeadingInterpolation(shootBall5.getHeading(), gateCollect4.getHeading())
                .build();
        shoot7 = follower.pathBuilder()
                .addPath(new BezierCurve(gateCollect4, shootPose2ToGateControlPoint, shootBall7))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new BezierLine(shootBall7, collect3end))
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
                theWheelOfTheOx.setPower(0);
                launcher.setVelocity(1100);
                tree.setPower(1);
                blocker.setPosition(0);
                launcher.setVelocity(1100); //1725
                hood.setPosition(1); //0.285
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(shoot1);
                if (pathTimer.getElapsedTimeSeconds()>0.5) {
                    theWheelOfTheOx.setPower(-1);
                }

                setPathState(blue21close.PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                launcher.setVelocity(1100);
                if(pathTimer.getElapsedTimeSeconds()>2.1)
                {
                    blocker.setPosition(1);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2.2){
                    tree.setPower(1);
                    launcher.setVelocity(1100);
                    hood.setPosition(1);
                    theWheelOfTheOx.setPower(-1);
                    if(pathTimer.getElapsedTimeSeconds()>2.2)
                    {
                        launcher.setVelocity(1100);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>2.6) {
                        setPathState(blue21close.PathState.collection);
                    }
                }
                break;


            case collection:
                hood.setPosition(1);
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(0);
                tree.setPower(1);
                if (!follower.isBusy() && !collectionStarted) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    launcher.setVelocity(1100);
                    hood.setPosition(1);
                    tree.setPower(1);
                    follower.followPath(collect1);
                    collectionStarted = true;
                    if (pathTimer.getElapsedTimeSeconds()>1.25)
                    {
                        theWheelOfTheOx.setPower(-1);
                    }
                }
                if (!follower.isBusy() && collectionStarted) {
                    setPathState((blue21close.PathState.shoot));


                }
                break;
            case shoot:
                if (!follower.isBusy() && !shoot2Started) {
                    hood.setPosition(1);
                    follower.followPath(shoot2);
                    launcher.setVelocity(1100);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot2Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot2Started) {
                    if (pathTimer.getElapsedTimeSeconds()>2)
                    {
                        blocker.setPosition(1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>2.25) {
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>2.85) {
                        setPathState((blue21close.PathState.GateCollection));
                    }
                }
                break;

            case GateCollection:
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(0);
                if (!follower.isBusy() && !gateCollectionStarted) {
                    follower.followPath(GateCollect1);
                    launcher.setVelocity(1120);
                    tree.setPower(1);
                    hood.setPosition(1);

                    gateCollectionStarted = true; // Mark as started to prevent calling again
                }
                if (pathTimer.getElapsedTimeSeconds()>2.25) {
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && gateCollectionStarted && pathTimer.getElapsedTimeSeconds()>3) {
                    setPathState((blue21close.PathState.shootAgain));
                }
                break;
            case shootAgain:
                if (!follower.isBusy() && !shoot3Started) {
                    hood.setPosition(1);
                    follower.followPath(shoot3);
                    tree.setPower(1);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    //tree.setPower(1);
                    shoot3Started = true; // Mark as started to prevent calling again
                }
                if(pathTimer.getElapsedTimeSeconds()>1)
                {
                    tree.setPower(0);
                }
                if(pathTimer.getElapsedTimeSeconds()>1.65)
                {
                    tree.setPower(1);
                    blocker.setPosition(1);
                }
                if (!follower.isBusy() && shoot3Started) {
                    if(pathTimer.getElapsedTimeSeconds()>1.8) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>2.65)
                    {
                        setPathState((blue21close.PathState.GateCollectionAgain));
                    }
                }
                break;

            case GateCollectionAgain:
                if (!follower.isBusy() && !gateCollectionAgainStarted) {
                    blocker.setPosition(0);
                    theWheelOfTheOx.setPower(0);
                    follower.followPath(GateCollect2);
                    hood.setPosition(1);
                    launcher.setVelocity(1120);
                    tree.setPower(1);
                    hood.setPosition(1);
                    gateCollectionAgainStarted = true; // Mark as started to prevent calling again
                }
                if (pathTimer.getElapsedTimeSeconds()>2.25) {
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && gateCollectionAgainStarted && pathTimer.getElapsedTimeSeconds()>3.15) {
                    setPathState((blue21close.PathState.shootAgainAgain));
                }
                break;
            case shootAgainAgain:
                tree.setPower(1);
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && !shoot4Started) {
                    hood.setPosition(1);
                    follower.followPath(shoot4);
                    tree.setPower(1);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    shoot4Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot4Started) {
                    if(pathTimer.getElapsedTimeSeconds()>0.9)
                    {
                        tree.setPower(0);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.45)
                    {
                        tree.setPower(1);
                        blocker.setPosition(1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.6) {
                        blocker.setPosition(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>2.65)
                    {
                        setPathState((blue21close.PathState.GateCollectionAgainAgain));
                    }
                }
                break;
            case GateCollectionAgainAgain:
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(0);
                if (!follower.isBusy() && !gateCollectionAgainStarted) {
                    follower.followPath(GateCollect3);
                    launcher.setVelocity(1120);
                    tree.setPower(1);
                    hood.setPosition(1);
                    gateCollectionAgainStarted = true; // Mark as started to prevent calling again
                }
                if (pathTimer.getElapsedTimeSeconds()>2.25) {
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && gateCollectionAgainStarted && pathTimer.getElapsedTimeSeconds()>3.55) {
                    // In 18-close we would go straight to shootAgainAgainAgain here.
                    // For 21-close, insert an extra gate cycle first.
                    setPathState((PathState.shootAgainAgainAgain));
                }
                break;
            case shootAgainAgainAgain:
                tree.setPower(1);
                // Start path once only – use shoot5Started (was wrongly setting shoot4Started)
                if (!follower.isBusy() && !shoot5Started) {
                    hood.setPosition(1);
                    follower.followPath(shoot5);
                    tree.setPower(1);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    shoot5Started = true;
                }
                // Run shooting timers by time – don’t wait for !follower.isBusy() so oscillation doesn’t block
                if (shoot5Started) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.9) tree.setPower(0);
                    if (pathTimer.getElapsedTimeSeconds() > 1.45) {
                        tree.setPower(1);
                        blocker.setPosition(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                        blocker.setPosition(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.65) {
                        setPathState((PathState.GateCollectionAgainAgainExtra));
                    }
                }
                break;

            case GateCollectionAgainAgainExtra:
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(0);
                if (!follower.isBusy() && !gateCollectionAgainAgainExtraStarted) {
                    follower.followPath(GateCollect4);
                    launcher.setVelocity(1120);
                    tree.setPower(1);
                    hood.setPosition(1);
                    gateCollectionAgainAgainExtraStarted = true;
                }
                if (pathTimer.getElapsedTimeSeconds()>2.25) {
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && gateCollectionAgainAgainExtraStarted && pathTimer.getElapsedTimeSeconds()>2.65) {
                    setPathState(PathState.shootAgainAgainAgainExtra);
                }
                break;

            case shootAgainAgainAgainExtra:
                tree.setPower(1);
                // Start path once only (no re-call when oscillating) – same pattern as other shoot states
                if (!follower.isBusy() && !shoot6Started) {
                    follower.followPath(shoot7);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot6Started = true;
                }
                // Run shooting timers by time only – don’t wait for !follower.isBusy() so oscillation doesn’t block progress
                if (shoot6Started) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.9) tree.setPower(0);
                    if (pathTimer.getElapsedTimeSeconds() > 1.35) {
                        tree.setPower(1);
                        blocker.setPosition(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        blocker.setPosition(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 2.55) {
                        setPathState((blue21close.PathState.collectAgainAgainEnd));
                    }
                }
                break;

            case collectAgainAgainEnd:
                blocker.setPosition(0);
                theWheelOfTheOx.setPower(0);
                if(pathTimer.getElapsedTimeSeconds()>1.5)
                {
                    theWheelOfTheOx.setPower(-1);
                }
                if (!follower.isBusy() && !collectionStarted) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    launcher.setVelocity(1080);
                    hood.setPosition(1);
                    tree.setPower(1);
                    follower.followPath(collect3);
                    collectionStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && collectionStarted) {
                    theWheelOfTheOx.setPower(-1);
                        setPathState((blue21close.PathState.shootAgainAgainAgainAgain));

                }
                break;
            case shootAgainAgainAgainAgain:
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && !shoot7Started) {
                    follower.followPath(shoot6);
                    launcher.setVelocity(1080);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(0);
                    shoot7Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot7Started) {
                    if(pathTimer.getElapsedTimeSeconds()>1.5)
                    {
                        blocker.setPosition(1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.75) {
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>2.65) {
                        setPathState((blue21close.PathState.done));
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
        gateCollectionAgainAgainExtraStarted = false;
        shoot6Started = false;
        shoot7Started=false;
    }

    @Override
    public void init() {
        pathState = PathState.start;
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0, 0.362);
        blocker.setPosition(0);
        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0,0.0761);
        hood.setPosition(1);
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
        hood = hardwareMap.get(Servo.class, "hood");
        //hood.setPosition(0.0119);
        hood.scaleRange(0,0.0761);
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0, 0.4);

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theWheelOfTheOx.setDirection(DcMotorSimple.Direction.FORWARD);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double P = 400;
        double I = 0;
        double D = 0;
        double F = 13.2965;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        follower.update();

        // Rotator localization on the whole time: always aim at blue goal from current pose (while driving and when stopped)
        if (turret != null) aimRotatorAtBlueGoal();

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
        if (turret == null) return;
        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        // Blue side uses +offset instead of -offset
        int newPosition = turret.getRotatorPos() + adjustment + offset;
        turret.setRotatorPos(newPosition);
    }

    /** Turret angle (deg) to aim at blue goal from current pose. */
    private double getTurretAngleDegForBlueGoal() {
        Pose robot = follower.getPose();
        double dx = BLUE_GOAL_X - robot.getX();
        double dy = BLUE_GOAL_Y - robot.getY();
        double angleToGoalDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg = Math.toDegrees(robot.getHeading());
        double turretAngleDeg = angleToGoalDeg - headingDeg;
        while (turretAngleDeg > 180) turretAngleDeg -= 360;
        while (turretAngleDeg < -180) turretAngleDeg += 360;
        return -turretAngleDeg + BLUE_AIM_OFFSET_DEG;
    }

    /** Aims the rotator at the blue goal using pose (localization). */
    private void aimRotatorAtBlueGoal() {
        if (turret == null) return;
        turret.setRotatorToAngle(getTurretAngleDegForBlueGoal());
    }

    /** True when chassis speed is below threshold so we can lock on. */
    private boolean isRobotAtRest() {
        Vector v = follower.getVelocity();
        return v != null && v.getMagnitude() < CHASSIS_AT_REST_THRESHOLD;
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

}

