package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "Red 15 close ", group = "new bot")
public class red15closenewbot extends OpMode {
    private int rotatorStartPosition=0;
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

    }

    PathState pathState;
    private final Pose startPose = new Pose(118, 132.2, Math.toRadians(36.5));
    private final Pose shootPose1 = new Pose(88, 92, Math.toRadians(46));
    private final Pose collect1thingstart = new Pose(88, 59, Math.toRadians(0));
    private final Pose collect1thing = new Pose(127, 59, Math.toRadians(0));
    private final Pose shootPose2 = new Pose( 89, 88, Math.toRadians(48.5));
    private final Pose gateCollect1 = new Pose( 128, 62.75, Math.toRadians(22.5));
    private final Pose inBetween1 = new Pose(100, 62.75, Math.toRadians(22.5));
    private final Pose shootBall3 = new Pose(89, 88, Math.toRadians(45));
    private final Pose inBetween2 = new Pose(100, 62.75, Math.toRadians(22.5));
    private final Pose gateCollect2 = new Pose( 128, 62.75, Math.toRadians(22.));
    private final Pose shootBall4 = new Pose(89, 88, Math.toRadians(45));
    private final Pose collect3start=new Pose(87, 86, Math.toRadians(0));

    //
    private final Pose collect3end = new Pose(123, 86, Math.toRadians(0));
    private final Pose shootBall5 = new Pose(95, 115, Math.toRadians(30));

    private final Pose park = new Pose(103, 84, Math.toRadians(46));



    private PathChain shoot1, goToCollect1, collect1, shoot2,InBetween1, InBetween2, GateCollect1, GateCollect2, shoot3, awayfromGate, goToCollect3, collect3, shoot4, goToGate, openGate, goToCollect4, collect4, shoot5, parking;

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
        InBetween1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, inBetween1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), inBetween1.getHeading())
                .build();

        GateCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(inBetween1, gateCollect1))
                .setLinearHeadingInterpolation(inBetween1.getHeading(), gateCollect1.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(gateCollect1, shootBall3))
                .setLinearHeadingInterpolation(gateCollect1.getHeading(), shootBall3.getHeading())
                .build();
        InBetween2 = follower.pathBuilder()
                .addPath(new BezierLine(shootBall3, inBetween2))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), inBetween2.getHeading())
                .build();
        GateCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(inBetween2, gateCollect2))
                .setLinearHeadingInterpolation(inBetween2.getHeading(), gateCollect2.getHeading())
                .build();
        shoot4 = follower.pathBuilder()
                .addPath(new BezierLine(gateCollect2, shootBall4))
                .setLinearHeadingInterpolation(gateCollect2.getHeading(), shootBall4.getHeading())
                .build();

        goToCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(shootBall3, collect3start))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), collect3start.getHeading())
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new BezierLine(collect3start, collect3end))
                .setLinearHeadingInterpolation(collect3start.getHeading(), collect3end.getHeading())
                .build();
//
//
        shoot5 = follower.pathBuilder()
                .addPath(new BezierLine(collect3end, shootBall5))
                .setLinearHeadingInterpolation(collect3end.getHeading(), shootBall5.getHeading())
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
                tree.setPower(1);
                theWheelOfTheOx.setPower(0.8);
                rotator.setTargetPosition(rotatorStartPosition);
                // Try to use limelight for initial adjustment, fallback to hardcoded values
                launcher.setVelocity(1260); //1725
                //hood.setPosition(0.2); //0.285
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(shoot1);
                setPathState(red15closenewbot.PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                rotator.setTargetPosition(rotatorStartPosition);
                launcher.setVelocity(1260);
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2){
                    tree.setPower(1);
                    launcher.setVelocity(1260);
                    rotator.setTargetPosition(rotatorStartPosition);
                    theWheelOfTheOx.setPower(-1);
                    if (pathTimer.getElapsedTimeSeconds()>3.5) {
                        setPathState(red15closenewbot.PathState.collection);
                    }
                }
                break;
            case gotocollect:
                rotator.setTargetPosition(rotatorStartPosition);
                if(!follower.isBusy())
                {
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    follower.followPath(goToCollect1);
                    setPathState(red15closenewbot.PathState.collection);
                }
                break;


            case collection:
                rotator.setTargetPosition(rotatorStartPosition);
                if (!follower.isBusy() && !collectionStarted) {
                    //rotator.setTargetPosition(rotatorStartPosition);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    launcher.setVelocity(1280);
                    //hood.setPosition(0.2);
                    theWheelOfTheOx.setPower(0.8);
                    tree.setPower(1);
                    follower.followPath(collect1);
                    collectionStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && collectionStarted) {
                    setPathState((PathState.shoot));
                }
                break;
            case shoot:
                rotator.setTargetPosition(rotatorStartPosition);
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && !shoot2Started) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot2);
                    launcher.setVelocity(1280);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot2Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot2Started) {
                    if(pathTimer.getElapsedTimeSeconds()>2.5) {
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3.5) {
                        setPathState((PathState.beginGateCollection));
                    }
                }
                break;

            case beginGateCollection:
                rotator.setTargetPosition(rotatorStartPosition);
                if (!follower.isBusy() && !beginGateCollectionStarted) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    follower.followPath(InBetween1);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    beginGateCollectionStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && beginGateCollectionStarted) {
                    setPathState((PathState.GateCollection));
                }
                break;
            case GateCollection:
                rotator.setTargetPosition(rotatorStartPosition);
                if (!follower.isBusy() && !gateCollectionStarted) {
                    follower.followPath(GateCollect1);
                    rotator.setTargetPosition(rotatorStartPosition);
                    launcher.setVelocity(1260);
                    tree.setPower(1);
                    //hood.setPosition(0.2);
                    theWheelOfTheOx.setPower(0.8);
                    //theWheelOfTheOx.setPower(0.005);
                    //hood.setPosition(0.225);
                    gateCollectionStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && gateCollectionStarted && pathTimer.getElapsedTimeSeconds()>3.25) {
                    setPathState((red15closenewbot.PathState.shootAgain));
                }
                break;
            case shootAgain:
                rotator.setTargetPosition(rotatorStartPosition);
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && !shoot3Started) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot3);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot3Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot3Started) {
                    if(pathTimer.getElapsedTimeSeconds()>2) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3.5)
                    {
                        setPathState((PathState.beginGateCollectionAgain));
                    }
                }
                break;
            case beginGateCollectionAgain:
                if (!follower.isBusy() && !beginGateCollectionAgainStarted) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    follower.followPath(InBetween2);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    beginGateCollectionAgainStarted = true; // Mark as started to prevent calling again
                    rotator.setTargetPosition(rotatorStartPosition);
                }
                if (!follower.isBusy() && beginGateCollectionAgainStarted) {
                    setPathState((PathState.GateCollectionAgain));
                }
                break;
            case GateCollectionAgain:
                if (!follower.isBusy() && !gateCollectionAgainStarted) {
                    follower.followPath(GateCollect2);
                    rotator.setTargetPosition(rotatorStartPosition);
                    launcher.setVelocity(1260);
                    tree.setPower(1);
                    //hood.setPosition(0.2);
                    theWheelOfTheOx.setPower(0.8);
                    //theWheelOfTheOx.setPower(0.005);
                    //hood.setPosition(0.225);
                    gateCollectionAgainStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && gateCollectionAgainStarted && pathTimer.getElapsedTimeSeconds()>3.25) {
                    setPathState((red15closenewbot.PathState.shootAgainAgain));
                }
                break;
            case shootAgainAgain:
                tree.setPower(1);
                rotator.setTargetPosition(rotatorStartPosition);
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && !shoot4Started) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot4);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot4Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot4Started) {
                    if(pathTimer.getElapsedTimeSeconds()>2.5) {
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3.5)
                    {
                        setPathState((PathState.collectAgainAgain));
                    }
                }
                break;
            case collectAgainAgain:
                rotator.setTargetPosition(rotatorStartPosition);
                if(!follower.isBusy())
                {
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    follower.followPath(goToCollect3);
                    setPathState(PathState.collectAgainAgainEnd);
                }
                break;


            case collectAgainAgainEnd:
                rotator.setTargetPosition(rotatorStartPosition);
                if (!follower.isBusy() && !collectionStarted) {
                    //rotator.setTargetPosition(rotatorStartPosition);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    launcher.setVelocity(1260);
                    //hood.setPosition(0.2);
                    theWheelOfTheOx.setPower(0.8);
                    tree.setPower(1);
                    follower.followPath(collect3);
                    collectionStarted = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && collectionStarted) {
                    setPathState((PathState.shootAgainAgainAgain));
                }
                break;
            case shootAgainAgainAgain:
                rotator.setTargetPosition(rotatorStartPosition);
                // Continuously adjust based on limelight during shooting
                if (!follower.isBusy() && !shoot5Started) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot5);
                    launcher.setVelocity(1240);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(1);
                    shoot5Started = true; // Mark as started to prevent calling again
                }
                if (!follower.isBusy() && shoot5Started) {
                    if(pathTimer.getElapsedTimeSeconds()>2) {
                        tree.setPower(1);
                        theWheelOfTheOx.setPower(-1);
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3.5) {
                        setPathState((PathState.done));
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
        //hood.setPosition(0.0119);
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
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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