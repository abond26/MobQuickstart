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

@Autonomous(name = "Reliable 9 blue far new bot", group = "new bot")
public class blue9farnewbot extends OpMode {
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
    // Mirrored from red: X -> 144 - X, heading -> Math.PI - heading
    private final Pose startPose = new Pose(56.7, 7.5, Math.toRadians(90));                    // red (87, 7, 90)
    private final Pose shootPose1 = new Pose(55, 15, Math.toRadians(108));         // red (89, 15, 72)
    //private final Pose collect1thingstart = new Pose(54, 41, Math.PI);                      // red (90, 41, 0)
    private final Pose collect1ControlPoint = new Pose(47.80926430517712, 35.68937329700272); // red (96.19..., 35.69...)

    private final Pose collect1thing = new Pose(13, 36, Math.toRadians(180));                           // red (131, 36, 0)
    private final Pose shootPose2 = new Pose(55, 15, Math.toRadians(114));         // red (89, 15, 66)

    private final Pose collect2End = new Pose(11, 12, Math.toRadians(180));// red (135, 4, 0)
    private final Pose collect2Start2 = new Pose(15, 6, Math.toRadians(180));// red (135, 4, 0)

    private final Pose collect2End2 = new Pose(12, 6, Math.toRadians(180));// red (135, 4, 0)

    private final Pose shootBall3 = new Pose(55, 15, Math.toRadians(112));         // red (89, 10, 64)
    private final Pose shoot3ControlPoint = new Pose(30.215258855585834, 20.50544959128065); // red (106.53..., 8.24...)

    private final Pose park = new Pose(41, 22, Math.PI - Math.toRadians(64));               // red (103, 22, 64)


    private PathChain shoot1, goToCollect1, collect2StartAgain, collect2EndAgain, collect1, shoot2, goToCollect2, collect2, shoot3,goToCollect2Again, collect2Again, goToCollect2AgainAgain, collect2AgainAgain, goToCollect3, collect3, shoot4, goToCollect4, collect4, shoot5, parking;

    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();

        collect1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, collect1ControlPoint, collect1thing))
                .setTangentHeadingInterpolation()
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
                .addPath(new BezierLine(collect2End, collect2Start2))
                .setLinearHeadingInterpolation(collect2End.getHeading(), collect2Start2.getHeading())
                .build();
        collect2EndAgain = follower.pathBuilder()
                .addPath(new BezierLine(collect2Start2, collect2End2))
                .setLinearHeadingInterpolation(collect2Start2.getHeading(), collect2End2.getHeading())
                .build();

        shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(collect2End2, shoot3ControlPoint, shootBall3))
                .setLinearHeadingInterpolation(collect2End2.getHeading(), shootBall3.getHeading())
                .build();
        parking = follower.pathBuilder()
                .addPath(new BezierLine(shootBall3, park))
                .setLinearHeadingInterpolation(shootBall3.getHeading(), park.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                hood.setPosition(0);
                blocker.setPosition(0);
                launcher.setVelocity(1500);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                rotator.setTargetPosition(rotatorStartPosition);
                follower.followPath(shoot1);
                setPathState(PathState.actuallyshoot1);
                break;
            case actuallyshoot1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2.5) {
                    blocker.setPosition(1);
                    tree.setPower(0.6);
                    rotator.setTargetPosition(rotatorStartPosition);
                    //theWheelOfTheOx.setPower(-1);
                    launcher.setVelocity(1500);
                }
                if (pathTimer.getElapsedTimeSeconds()>2.75)
                {
                    theWheelOfTheOx.setPower(-1);
                    launcher.setVelocity(1500);
                }
                if (pathTimer.getElapsedTimeSeconds()>4.5) {
                        setPathState((PathState.collection));
                    }
                break;

            case collection:
                blocker.setPosition(0);
                if (!follower.isBusy()) {
                    follower.followPath(collect1);
                    launcher.setVelocity(1500);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    tree.setPower(1);
                    setPathState((blue9farnewbot.PathState.shoot));
                }
                break;
            case shoot:
                if (!follower.isBusy() && !shoot2Started) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    launcher.setVelocity(1500);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(0.6);
                    follower.followPath(shoot2);
                    shoot2Started = true;
                }
                if (!follower.isBusy() && shoot2Started) {
                    if (pathTimer.getElapsedTimeSeconds()>3.75)
                    {
                        blocker.setPosition(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>4)
                    {
                        theWheelOfTheOx.setPower(-1);
                        launcher.setVelocity(1500);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>6) {
                        setPathState((PathState.collectAgainEnd));
                    }
                }
                break;
            case collectAgainEnd:
                tree.setPower(1);
                blocker.setPosition(0);
                rotator.setTargetPosition(rotatorStartPosition);
                if (!follower.isBusy() && !collectAgainEndStarted) {
                    follower.followPath(collect2);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(0);
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    collectAgainEndStarted = true;
                }
                if (!follower.isBusy() && collectAgainEndStarted) {
                    setPathState((PathState.collectAgainAgain));
                }
                break;
            case collectAgainAgain:
                tree.setPower(1);
                rotator.setTargetPosition(rotatorStartPosition);
                blocker.setPosition(0);
                if (!follower.isBusy() && !collectAgainAgainStarted) {
                    follower.followPath(collect2Again);
                    rotator.setTargetPosition(rotatorStartPosition);
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(0);
                    tree.setPower(1);
                    collectAgainAgainStarted = true;
                }
                if (!follower.isBusy() && collectAgainAgainStarted) {
                    rotator.setTargetPosition(rotatorStartPosition);
                    setPathState((PathState.collectAgainAgainEnd));
                }

                break;
            case collectAgainAgainEnd:
                tree.setPower(1);
                rotator.setTargetPosition(rotatorStartPosition);
                blocker.setPosition(0);
                if (!follower.isBusy() && !collectAgainAgainEndStarted) {
                    follower.followPath(collect2EndAgain);
                    rotator.setTargetPosition(rotatorStartPosition);
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
                launcher.setVelocity(1500);
                if (!follower.isBusy() && !shoot3Started) {
                    launcher.setVelocity(1500);
                    rotator.setTargetPosition(rotatorStartPosition);
                    follower.followPath(shoot3);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    tree.setPower(0.6);
                    shoot3Started = true;
                }
                if (!follower.isBusy() && shoot3Started) {
                    if (pathTimer.getElapsedTimeSeconds()>2)
                    {
                        blocker.setPosition(1);
                    }
                    if (pathTimer.getElapsedTimeSeconds()>2.25)
                    {
                        theWheelOfTheOx.setPower(-1);
                        launcher.setVelocity(1500);
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
                    parkingStarted = true;
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
        double P = 270;
        double I = 0;
        double D = 0;
        double F = 13.2965;
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
    /** Blue side: rotator adjustment uses + offset (mirrored from red's - offset). */
    public void adjustRotator(double tx) {
        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        int newPosition = rotator.getCurrentPosition() + adjustment + offset;
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
