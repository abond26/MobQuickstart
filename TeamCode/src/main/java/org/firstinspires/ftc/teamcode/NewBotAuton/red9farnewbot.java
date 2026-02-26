package org.firstinspires.ftc.teamcode.NewBotAuton;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;

@Autonomous(name = "Reliable 9 red far new bot", group = "new bot")
public class red9farnewbot extends OpMode {
    private int rotatorStartPosition=0;
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
        parklol,


        done,

        VERYYYY_THIRD_INTAKE,

        THIRD_SHOT_PREP,

        PAUSE3,

        SHOT_3,

        JACK_OFF,

    }


    PathState pathState;
    private final Pose startPose = new Pose(87, 7, Math.toRadians(90));
    private final Pose shootPose1 = new Pose(89, 15, Math.toRadians(72));
    private final Pose collect1thingstart=new Pose(90, 41, Math.toRadians(0));
    private final Pose collect1ControlPoint=new Pose(96.19073569482288, 35.68937329700272);

    private final Pose collect1thing = new Pose(131, 36, Math.toRadians(0));
    private final Pose shootPose2 = new Pose( 89, 15, Math.toRadians(66));

    private final Pose collect2ControlPoint=new Pose(106.52861035422343, 8.239782016348768);

    //private final Pose collect2Start = new Pose(120, 15, Math.toRadians(0));
    private final Pose collect2End = new Pose(135, 4, Math.toRadians(0));
//    private final Pose collect2StartAgain = new Pose(120, 17, Math.toRadians(0));
//    private final Pose collect2EndAgain = new Pose(125, 17, Math.toRadians(0));
//    private final Pose collect2StartAgainAgain = new Pose(120, 17, Math.toRadians(0));
//    private final Pose collect2EndAgainAgain = new Pose(125, 17, Math.toRadians(0));
    private final Pose shootBall3 = new Pose(89, 10, Math.toRadians(64));
    private final Pose park = new Pose(103, 22, Math.toRadians(64));


    private PathChain shoot1, goToCollect1, collect1, shoot2, goToCollect2, collect2, shoot3,goToCollect2Again, collect2Again, goToCollect2AgainAgain, collect2AgainAgain, goToCollect3, collect3, shoot4, goToCollect4, collect4, shoot5, parking;

    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

//        goToCollect1 = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose1, collect1thingstart))
//                .setLinearHeadingInterpolation(shootPose1.getHeading(), collect1thingstart.getHeading())
//                .build();

        collect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, collect1ControlPoint, collect1thing))
                .setTangentHeadingInterpolation()
                .build();



        shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(collect1thing, shootPose2))
                .setLinearHeadingInterpolation(collect1thing.getHeading(), shootPose2.getHeading())
                .build();


//        goToCollect2 = follower.pathBuilder()
//                .addPath(new BezierLine(shootPose2, collect2Start))
//                .setLinearHeadingInterpolation(shootPose2.getHeading(), collect2Start.getHeading())
//                .build();

        collect2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2,collect2ControlPoint, collect2End))
                .setTangentHeadingInterpolation()
                .build();

//        goToCollect2Again = follower.pathBuilder()
//                .addPath(new BezierLine(collect2End, collect2StartAgain))
//                .setLinearHeadingInterpolation(collect2End.getHeading(), collect2StartAgain.getHeading())
//                .build();
//
//        collect2Again = follower.pathBuilder()
//                .addPath(new BezierLine(collect2StartAgain, collect2EndAgain))
//                .setLinearHeadingInterpolation(collect2StartAgain.getHeading(), collect2EndAgain.getHeading())
//                .build();
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
                .addPath(new BezierLine(collect2End, shootBall3))
                .setLinearHeadingInterpolation(collect2End.getHeading(), shootBall3.getHeading())
                .build();
        parking=follower.pathBuilder()
                .addPath(new BezierLine(shootBall3, park))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), park.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                hood.setPosition(0.25);
                blocker.setPosition(0);
                // Try to use limelight for initial adjustment, fallback to hardcoded values
                launcher.setVelocity(1600);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                rotator.setTargetPosition(rotatorStartPosition);
                follower.followPath(shoot1);
                setPathState(PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2.5) {
                    blocker.setPosition(1);
                    tree.setPower(1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    theWheelOfTheOx.setPower(-1);
                    launcher.setVelocity(1600);
                }
                if (pathTimer.getElapsedTimeSeconds()>2.75)
                {
                    blocker.setPosition(0);
                    theWheelOfTheOx.setPower(0);
                    launcher.setVelocity(1600);
                }
                if (pathTimer.getElapsedTimeSeconds()>3)
                {
                    blocker.setPosition(1);
                    theWheelOfTheOx.setPower(-1);
                    launcher.setVelocity(1600);
                }
                if (pathTimer.getElapsedTimeSeconds()>3.25)
                {
                    blocker.setPosition(0);
                    theWheelOfTheOx.setPower(-1);
                    launcher.setVelocity(1600);
                }
                if (pathTimer.getElapsedTimeSeconds()>3.5)
                {
                    blocker.setPosition(1);
                    theWheelOfTheOx.setPower(-1);
                    launcher.setVelocity(1600);
                }
                if (pathTimer.getElapsedTimeSeconds()>4.5) {
                        setPathState((PathState.collection));
                    }
                break;
//            case gotocollect:
//                if(!follower.isBusy())
//                {
//                    blocker.setPosition(0);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    launcher.setVelocity(1640);
//                    follower.followPath(goToCollect1);
//                    theWheelOfTheOx.setPower(0);
//                    setPathState(PathState.collection);
//                }
//                break;


            case collection:
                blocker.setPosition(0);
                if (!follower.isBusy()) {
                    launcher.setVelocity(1600);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    tree.setPower(1);
                    follower.followPath(collect1);
                    setPathState((red9farnewbot.PathState.shoot));
                }
                if (pathTimer.getElapsedTimeSeconds()>2.25)
                {
                    theWheelOfTheOx.setPower(-1);
                }
                break;
            case shoot:
                if (!follower.isBusy() && !shoot2Started) {
                    follower.followPath(shoot2);
                    rotator.setTargetPosition(rotatorStartPosition);
                    launcher.setVelocity(1600);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot2Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot2Started) {
                    if (pathTimer.getElapsedTimeSeconds()>3.25)
                    {
                        blocker.setPosition(0);
                        theWheelOfTheOx.setPower(0);
                        launcher.setVelocity(1600);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>3.5)
                    {
                        blocker.setPosition(1);
                        theWheelOfTheOx.setPower(-1);
                        launcher.setVelocity(1600);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>3.75)
                    {
                        blocker.setPosition(0);
                        theWheelOfTheOx.setPower(-1);
                        launcher.setVelocity(1600);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>4)
                    {
                        blocker.setPosition(1);
                        theWheelOfTheOx.setPower(-1);
                        launcher.setVelocity(1600);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>5) {
                        setPathState((PathState.collectAgainEnd));
                    }
                }
                break;
            case collectAgainEnd:
                blocker.setPosition(0);
                if (!follower.isBusy() && !collectAgainEndStarted) {
                    follower.followPath(collect2);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(0);
                    tree.setPower(1);
                    collectAgainEndStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && collectAgainEndStarted) {
                    setPathState((PathState.shootAgain));
                }
                if (pathTimer.getElapsedTimeSeconds()>1)
                {
                    theWheelOfTheOx.setPower(-1);
                }
                break;
            case shootAgain:
                launcher.setVelocity(1600);
                if (!follower.isBusy() && !shoot3Started) {
                    launcher.setVelocity(1600);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot3);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot3Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot3Started) {

                    if (pathTimer.getElapsedTimeSeconds()>2.75)
                    {
                        blocker.setPosition(1);
                        theWheelOfTheOx.setPower(-1);
                        launcher.setVelocity(1600);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>3)
                    {
                        blocker.setPosition(0);
                        theWheelOfTheOx.setPower(-1);
                        launcher.setVelocity(1600);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>3.25)
                    {
                        blocker.setPosition(1);
                        theWheelOfTheOx.setPower(-1);
                        launcher.setVelocity(1600);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>4.25) {
                        setPathState((PathState.parklol));
                    }
                }
                break;
            case parklol:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1 && !parkingStarted) {
                    theWheelOfTheOx.setPower(-1);
                    follower.followPath(parking);
                    parkingStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && parkingStarted && pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState((PathState.done));
                }
                break;
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
        collectAgainStarted = false;
        collectAgainEndStarted = false;
        collectAgainAgainStarted = false;
        collectAgainAgainEndStarted = false;
        collectAgainAgainAgainStarted = false;
        collectAgainAgainAgainEndStarted = false;
        parkingStarted = false;
    }

    @Override
    public void init() {
        pathState = PathState.start;
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0, 0.4);
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


        tree = hardwareMap.get(DcMotor.class, "tree");
        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

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
        double P = 409;
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