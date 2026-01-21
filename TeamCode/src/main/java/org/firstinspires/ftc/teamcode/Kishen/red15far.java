package org.firstinspires.ftc.teamcode.Kishen;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.geometry.BezierLine;
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

@Autonomous(name = "Reliable 15 red far", group = "auton red")
public class red15far extends OpMode {
    private int rotatorStartPosition=0;
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
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

        THIRD_SHOT_PREP,

        PAUSE3,

        SHOT_3,

        JACK_OFF,

    }


    PathState pathState;
    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(89, 17, Math.toRadians(71.5));
    private final Pose collect1thingstart=new Pose(98, 33, Math.toRadians(0));


    private final Pose collect1thing = new Pose(127, 33, Math.toRadians(0));
    private final Pose shootPose2 = new Pose( 89, 17, Math.toRadians(71.5));
    private final Pose collect2Start = new Pose(88, 57.5, Math.toRadians(0));
    private final Pose collect2End = new Pose(128, 57.5, Math.toRadians(0));


//    private final Pose collect2Start = new Pose(130, 12, Math.toRadians(0));
//    private final Pose collect2End = new Pose(135, 12, Math.toRadians(0));
//    private final Pose collect2StartAgain = new Pose(125, 9, Math.toRadians(0));
//    private final Pose collect2EndAgain = new Pose(135, 9, Math.toRadians(0));
//    private final Pose collect2StartAgainAgain = new Pose(125, 10, Math.toRadians(0));
//    private final Pose collect2EndAgainAgain = new Pose(135, 10, Math.toRadians(0));
private final Pose openGateControlPoint = new Pose(113.89650349650348, 56.87412587412589);
private final Pose openGateStart = new Pose(120, 72, Math.toRadians(90));
    private final Pose openGateEnd = new Pose(125, 72, Math.toRadians(90));
    private final Pose shootBall3 = new Pose(89, 17, Math.toRadians(72));
    private final Pose collect3Start = new Pose(100, 9, Math.toRadians(0));
    private final Pose collect3End = new Pose(135, 9, Math.toRadians(0));
    private final Pose shootBall4 = new Pose(89, 13, Math.toRadians(71));
    private final Pose collect4Start = new Pose(100, 9, Math.toRadians(0));
    private final Pose collect4End = new Pose(135, 9, Math.toRadians(0));
    private final Pose shootBall5 = new Pose(89, 13, Math.toRadians(71.5));
    private final Pose park = new Pose(103, 22, Math.toRadians(73));


    private PathChain shoot1, goToCollect1, collect1, shoot2, goToCollect2, collect2, shoot3, opengatestart, opengateend, collect2Again, goToCollect2AgainAgain, collect2AgainAgain, goToCollect3, collect3, shoot4, goToCollect4, collect4, shoot5, parking;

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
                .addPath(new BezierCurve(collect2End,openGateControlPoint, openGateStart))
                .setLinearHeadingInterpolation(collect2End.getHeading(), openGateStart.getHeading())
                .build();

        opengateend = follower.pathBuilder()
                .addPath(new BezierLine(openGateStart, openGateEnd))
                .setLinearHeadingInterpolation(openGateStart.getHeading(), openGateEnd.getHeading())
                .build();
//        goToCollect2AgainAgain = follower.pathBuilder()
//                .addPath(new BezierLine(collect2EndAgain, collect2StartAgainAgain))
//                .setLinearHeadingInterpolation(collect2EndAgain.getHeading(), collect2StartAgainAgain.getHeading())
//                .build();
//
//        collect2AgainAgain = follower.pathBuilder()
//                .addPath(new BezierLine(collect2StartAgainAgain, collect2EndAgainAgain))
//                .setLinearHeadingInterpolation(collect2StartAgainAgain.getHeading(), collect2EndAgainAgain.getHeading())
//                .build();


        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(openGateEnd, shootBall3))
                .setLinearHeadingInterpolation(openGateEnd.getHeading(), shootBall3.getHeading())
                .build();
        goToCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(shootBall3, collect3Start))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), collect3Start.getHeading())
                .build();
        collect3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3Start, collect3End))
                .setLinearHeadingInterpolation(collect3Start.getHeading(), collect3End.getHeading())
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(collect3End, shootBall4))
                .setLinearHeadingInterpolation(collect3End.getHeading(), shootBall4.getHeading())
                .build();
        goToCollect4 = follower.pathBuilder()
                .addPath(new BezierLine(shootBall4, collect4Start))
                .setLinearHeadingInterpolation(shootBall4.getHeading(), collect4Start.getHeading())
                .build();
        collect4 = follower.pathBuilder()
                .addPath(new BezierLine(collect4Start, collect4End))
                .setLinearHeadingInterpolation(collect4Start.getHeading(), collect4End.getHeading())
                .build();
        shoot5 = follower.pathBuilder()
                .addPath(new BezierLine(collect4End, shootBall5))
                .setLinearHeadingInterpolation(collect4End.getHeading(), shootBall5.getHeading())
                .build();

        parking=follower.pathBuilder()
                .addPath(new BezierLine(shootBall5, park))
                .setLinearHeadingInterpolation(shootBall5.getHeading(), park.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                // Try to use limelight for initial adjustment, fallback to hardcoded values
                launcher.setVelocity(2100);
                hood.setPosition(0.397);
                rotator.setTargetPosition(rotatorStartPosition);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(shoot1);
                setPathState(PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>3.5){
                    tree.setPower(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    theWheelOfTheOx.setPower(-1);
                    if (pathTimer.getElapsedTimeSeconds()>4.5) {
                        setPathState(red15far.PathState.collection);
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
                    setPathState((red15far.PathState.shoot));
                }
                break;
            case shoot:
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && !shoot2Started) {
                    follower.followPath(shoot2);
                    rotator.setTargetPosition(rotatorStartPosition);
                    launcher.setVelocity(2050);
                    hood.setPosition(0.397);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot2Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot2Started) {
                    if(pathTimer.getElapsedTimeSeconds()>4) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>5) {
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
                    launcher.setVelocity(2175);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    setPathState((PathState.collectAgainEnd));
                }
                break;
            case collectAgainEnd:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(collect2);
                    tree.setPower(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    hood.setPosition(0.397);
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
                    setPathState(red15far.PathState.opengate);
                }
                break;
            case opengate:
                if (!follower.isBusy() && !opengateStarted) {
                    follower.followPath(opengateend);
                    opengateStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && opengateStarted && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(red15far.PathState.shootAgain);
                }
                break;
            case shootAgain:
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy()  && !shoot3Started) {
                    follower.followPath(shoot3);
                    launcher.setVelocity(2175);
                    hood.setPosition(0.397);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot3Started = true; // Mark as started to prevent calling again
                }
                //stuff needed
                if (!follower.isBusy() && shoot3Started) {
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>4)
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
                    hood.setPosition(0.397);
                    theWheelOfTheOx.setPower(1);
                    setPathState((PathState.shootAgainAgain));
                }
                break;
            case shootAgainAgain:
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy()  && !shoot4Started) {
                    follower.followPath(shoot4);
                    launcher.setVelocity(2075);
                    hood.setPosition(0.397);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    shoot4Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot4Started) {
                    if(pathTimer.getElapsedTimeSeconds()>4) {
                        theWheelOfTheOx.setPower(-1);
                        rotator.setTargetPosition(rotatorStartPosition);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>5)
                    {
                        setPathState(PathState.collectAgainAgainAgainEnd);
                    }
                }
                break;
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
                    hood.setPosition(0.397);
                    theWheelOfTheOx.setPower(1);
                    setPathState((PathState.shootAgainAgainAgain));
                }
                break;
            case shootAgainAgainAgain:
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy()  && !shoot5Started) {
                    follower.followPath(shoot5);
                    launcher.setVelocity(2075);
                    hood.setPosition(0.397);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot5Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot5Started) {
                    if(pathTimer.getElapsedTimeSeconds()>3) {
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
        rotatorStartPosition=0;
        rotator.setTargetPosition(rotatorStartPosition);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        int newPosition = rotator.getCurrentPosition() + adjustment - offset;
        rotator.setTargetPosition(newPosition);
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