package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;

//@Autonomous(name = "red 15 close new bot", group = "new bot")
public class red15closenewbot extends OpMode {
    private int rotatorStartPosition=0;
    private Follower follower;

    // Flags to prevent path oscillation - ensure paths are only called once per state
    private boolean shoot2Started = false;
    private boolean shoot3Started = false;
    private boolean shoot4Started = false;
    private boolean shoot5Started = false;
    private boolean goTowardsGateStarted = false;
    private boolean opengateStarted = false;

    private Servo hood;
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
    //Rotator var
    int motor180Range = 910;
    private int vMultiplier = 9;

    private DcMotor leftFront, leftRear, rightFront, rightRear;


    private DcMotorEx launcher;
    private DcMotor tree, theWheelOfTheOx, rotator;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
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
        goTowardsGate,
        opengate,
        parklol,


        done,

        VERYYYY_THIRD_INTAKE,
        inBetweenThing,
        THIRD_SHOT_PREP,

        PAUSE3,

        SHOT_3,

        JACK_OFF,

    }


    PathState pathState;
    private final Pose startPose = new Pose(117.6, 130, Math.toRadians(36.5));
    private final Pose shootPose1 = new Pose(85, 88, Math.toRadians(48));
    private final Pose collect1thingstart=new Pose(85, 84, Math.toRadians(0));


    private final Pose collect1thing = new Pose(120, 84, Math.toRadians(0));
    private final Pose shootPose2 = new Pose( 87, 84, Math.toRadians(49.5));

    private final Pose collect2Start = new Pose(88, 57.5, Math.toRadians(0));
    private final Pose collect2End = new Pose(128, 57.5, Math.toRadians(0));
    private final Pose openGateControlPoint = new Pose(102.61818181818181, 66.13846153846157);
    private final Pose openGateStart = new Pose(115, 72, Math.toRadians(90));
    private final Pose openGateEnd = new Pose(121, 72, Math.toRadians(90));

    private final Pose shootBall3 = new Pose(87, 84, Math.toRadians(52));
    private final Pose collect3start=new Pose(89, 35, Math.toRadians(0));

    //hello
    private final Pose collect3end = new Pose(122, 35, Math.toRadians(0));
    private final Pose shootBall4 = new Pose(87, 84, Math.toRadians(48.5));
    private final Pose collect4inbetween=new Pose(115, 55.5, Math.toRadians(-90));
    private final Pose collect4start=new Pose(128, 55.5, Math.toRadians(-90));


    private final Pose collect4end = new Pose(128, 12.5, Math.toRadians(-90));
    private final Pose shootBall5 = new Pose(85, 110, Math.toRadians(35));

    private final Pose park = new Pose(103, 84, Math.toRadians(46));


    private PathChain shoot1, goToCollect1, inBetweeen, collect1, shoot2, goToCollect2, collect2, shoot3, opengatestart, opengateend, collect2Again, goToCollect2AgainAgain, collect2AgainAgain, goToCollect3, collect3, shoot4, goToCollect4, collect4, shoot5, parking;

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

        opengatestart = follower.pathBuilder()
                .addPath(new BezierCurve(collect2End, openGateControlPoint, openGateStart))
                .setLinearHeadingInterpolation(collect2End.getHeading(), openGateStart.getHeading())
                .build();

        opengateend = follower.pathBuilder()
                .addPath(new BezierLine(openGateStart, openGateEnd))
                .setLinearHeadingInterpolation(openGateStart.getHeading(), openGateEnd.getHeading())
                .build();
        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(openGateEnd, shootBall3))
                .setLinearHeadingInterpolation(openGateEnd.getHeading(), shootBall3.getHeading())
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
        inBetweeen = follower.pathBuilder()
                .addPath(new BezierLine(shootBall4, collect4inbetween))
                .setLinearHeadingInterpolation(shootBall4.getHeading(), collect4inbetween.getHeading())
                .build();
        goToCollect4 = follower.pathBuilder()
                .addPath(new BezierLine(collect4inbetween, collect4start))
                .setLinearHeadingInterpolation(collect4inbetween.getHeading(), collect4start.getHeading())
                .build();
        collect4 = follower.pathBuilder()
                .addPath(new BezierLine(collect4start, collect4end))
                .setLinearHeadingInterpolation(collect4start.getHeading(), collect4end.getHeading())
                .build();
        shoot5 = follower.pathBuilder()
                .addPath(new BezierLine(collect4end, shootBall5))
                .setLinearHeadingInterpolation(collect4end.getHeading(), shootBall5.getHeading())
                .build();

        parking=follower.pathBuilder()
                .addPath(new BezierLine(shootBall5, park))
                .setLinearHeadingInterpolation(shootBall5.getHeading(), park.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                launcher.setVelocity(1725);
                hood.setPosition(0.285);
                rotator.setTargetPosition(rotatorStartPosition);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(shoot1);
                setPathState(PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                // Continuously set velocity to prevent override
                launcher.setVelocity(1725);
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2.5){
                    tree.setPower(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    theWheelOfTheOx.setPower(-1);
                    if (pathTimer.getElapsedTimeSeconds()>3.5) {
                        setPathState(red15closenewbot.PathState.collection);
                    }
                }
                break;
            case gotocollect:
                if(!follower.isBusy())
                {
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(goToCollect1);
                    setPathState(PathState.collection);
                }
                break;


            case collection:

                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    rotator.setTargetPosition(rotatorStartPosition);
                    theWheelOfTheOx.setPower(1);
                    tree.setPower(1);
                    follower.followPath(collect1);
                    setPathState((red15closenewbot.PathState.shoot));
                }
                break;
            case shoot:
                // Continuously set velocity to prevent override
                launcher.setVelocity(1725);
                if (!follower.isBusy() && !shoot2Started) {
                    follower.followPath(shoot2);
                    rotator.setTargetPosition(rotatorStartPosition);
                    hood.setPosition(0.285);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot2Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot2Started) {
                    if(pathTimer.getElapsedTimeSeconds()>3.5) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>4.5) {
                        setPathState((PathState.collectAgain));
                    }
                }
                break;
            case collectAgain:
                if (!follower.isBusy()) {
                    follower.followPath(goToCollect2);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    launcher.setVelocity(1700);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    setPathState((PathState.collectAgainEnd));
                }
                break;
            case collectAgainEnd:
                if (!follower.isBusy()) {
                    follower.followPath(collect2);
                    tree.setPower(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    hood.setPosition(0.3);
                    theWheelOfTheOx.setPower(1);
                    //theWheelOfTheOx.setPower(0.005);
                    //hood.setPosition(0.225);
                    setPathState((PathState.goTowardsGate));
                }
                break;
            case goTowardsGate:
                if (!follower.isBusy() && !goTowardsGateStarted) {
                    follower.followPath(opengatestart);
                    rotator.setTargetPosition(rotatorStartPosition);
                    goTowardsGateStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && goTowardsGateStarted) {
                    setPathState(red15closenewbot.PathState.opengate);
                }
                break;
            case opengate:
                if (!follower.isBusy() && !opengateStarted) {
                    follower.followPath(opengateend);
                    opengateStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && opengateStarted && pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(red15closenewbot.PathState.shootAgain);
                }
                break;
            case shootAgain:
                // Continuously set velocity to prevent override
                launcher.setVelocity(1725);
                if (!follower.isBusy()  && !shoot3Started) {
                    follower.followPath(shoot3);
                    hood.setPosition(0.285);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot3Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot3Started) {
                    if(pathTimer.getElapsedTimeSeconds()>2.5) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3.5)
                    {
                        setPathState(PathState.collectAgainAgainEnd);
                    }
                }
                break;
            case collectAgainAgain:
                if (!follower.isBusy()) {
                    follower.followPath(goToCollect3);
                    setPathState((PathState.collectAgainAgainEnd));

                }
            case collectAgainAgainEnd:
                if (!follower.isBusy()) {
                    follower.followPath(collect3);
                    tree.setPower(1);
                    hood.setPosition(0.285);
                    theWheelOfTheOx.setPower(1);
                    setPathState((PathState.shootAgainAgain));
                }
                break;
            case shootAgainAgain:
                // Continuously set velocity to prevent override
                launcher.setVelocity(1725);
                if (!follower.isBusy()  && !shoot4Started) {
                    follower.followPath(shoot4);
                    hood.setPosition(0.285);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    shoot4Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot4Started) {
                    if(pathTimer.getElapsedTimeSeconds()>3.5) {
                        theWheelOfTheOx.setPower(-1);
                        rotator.setTargetPosition(rotatorStartPosition);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>6)
                    {
                        setPathState(PathState.inBetweenThing);
                    }
                }
                break;
            case inBetweenThing:
                if (!follower.isBusy()) {
                    follower.followPath(inBetweeen);
                    setPathState((PathState.collectAgainAgainAgain));

                }
            case collectAgainAgainAgain:
                if (!follower.isBusy()) {
                    follower.followPath(goToCollect4);
                    setPathState((PathState.collectAgainAgainAgainEnd));

                }
            case collectAgainAgainAgainEnd:
                if (!follower.isBusy()) {
                    follower.followPath(collect4);
                    tree.setPower(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    hood.setPosition(0.285);
                    theWheelOfTheOx.setPower(1);
                    setPathState((PathState.shootAgainAgainAgain));
                }
                break;
            case shootAgainAgainAgain:
                // Continuously set velocity to prevent override
                launcher.setVelocity(1750);
                if (!follower.isBusy()  && !shoot5Started) {
                    follower.followPath(shoot5);
                    hood.setPosition(0.3);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot5Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot5Started) {
                    if(pathTimer.getElapsedTimeSeconds()>2.5) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>5)
                    {
                        setPathState(PathState.parklol);
                    }
                }
                break;
            case parklol:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(parking);
                    rotator.setTargetPosition(rotatorStartPosition);
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                        setPathState((PathState.done));
                    }
                }
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
        goTowardsGateStarted = false;
        opengateStarted = false;
    }

    @Override
    public void init() {
        pathState = PathState.start;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = ConstantsNewBot.createFollower(hardwareMap);
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
        hood.scaleRange(0,0.025);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorStartPosition=0;
        rotator.setTargetPosition(rotatorStartPosition);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set launcher mode for velocity control (RUN_WITHOUT_ENCODER for velocity control, even with encoder)
        //launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Initialize to target velocity to prevent default/previous values
        //launcher.setVelocity(1800);
        double P = 132.5;
        double I = 0;
        double D = 0;
        double F = 12.35;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
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
    }

    public void intake(double intakePower){
        tree.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-0.3);
        }
    }

}