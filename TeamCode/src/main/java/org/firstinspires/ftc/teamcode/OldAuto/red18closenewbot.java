//package org.firstinspires.ftc.teamcode.OldAuto;
//
//import com.pedropathing.follower.Follower;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.robotControl.RedUniversalConstants;
//import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Vision.Vision;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;
//import org.firstinspires.ftc.teamcode.util.PoseStorage;
//
//@Autonomous(name = "tangential Red 18 close ", group = "new bot", preselectTeleOp = "SmallManRed")
//public class red18closenewbot extends OpMode {
//    private int rotatorStartPosition=0;
//    double txDeg = 0.0; //horizontal deg
//    double tyDeg = 0.0; //vertical deg
//    private Follower follower;
//
//    private Vision vision = null;
//
//    // Flags to prevent path oscillation - ensure paths are only called once per state
//    private boolean shoot2Started = false;
//    private boolean shoot3Started = false;
//    private boolean shoot4Started = false;
//    private boolean shoot5Started = false;
//    private boolean goAwayFromGateStarted = false;
//    private boolean goTowardsGateStarted = false;
//    private boolean opengateStarted = false;
//    private boolean parkingStarted = false;
//    private boolean collectionStarted = false;
//    private boolean beginGateCollectionStarted = false;
//    private boolean gateCollectionStarted = false;
//    private boolean beginGateCollectionAgainStarted = false;
//    private boolean gateCollectionAgainStarted = false;
//
//    private Servo hood, blocker;
//    private int limeHeight = 33;
//    private int offset = 28;
//    private int tagHeight = 75;
//    private static final double NORMAL_DRIVE_POWER = 1;
//    private static final double INTAKE_DRIVE_POWER = 1; // tune this
//
//    // Hood adjustment constants (from TesterinoBlue)
//    private static final double DISTANCE_THRESHOLD = 180.0;
//    private static final double CLOSE_HOOD_POSITION = .2541; // Hood position for close shots
//    private static final double FAR_HOOD_POSITION = 0.36; // Hood position for far shots
//
//    private int y = tagHeight - limeHeight;
//    //Rotator var
//    int motor180Range = 910;
//    int limelightUpAngle = 25;
//    private int vMultiplier = 9;
//    private Limelight3A limelight;
//
//    // Store last valid limelight values for fallback
//    private double lastValidTx = 0.0;
//    private double lastValidTy = 0.0;
//    private double lastValidDistance = 0.0;
//    private boolean hasValidLimelightData = false;
//
//    private DcMotor leftFront, leftRear, rightFront, rightRear;
//
//
//    private DcMotorEx launcher;
//    private DcMotor tree, theWheelOfTheOx, rotator;
//    private Timer pathTimer, opModeTimer;
//
//    public enum PathState {
//        start,
//        actuallyshoot1,
//        gotocollect,
//        collection,
//        goAwayFromGate,
//        goTowardsGate,
//        opengate,
//        shoot,
//        collectAgain,
//        collectAgainEnd,
//
//
//        shootAgain,
//
//        collectAgainAgain,
//
//
//        collectAgainAgainEnd,
//
//        shootAgainAgain,
//
//        collectAgainAgainAgain,
//
//        collectAgainAgainAgainEnd,
//
//        shootAgainAgainAgain,
//        parklol,
//
//
//        done,
//
//        VERYYYY_THIRD_INTAKE,
//
//        THIRD_SHOT_PREP,
//
//        PAUSE3,
//
//        SHOT_3,
//
//        JACK_OFF,
//        beginGateCollection,
//        GateCollection,
//        beginGateCollectionAgain,
//        GateCollectionAgain,
//        GateCollectionAgainAgain,
//        shootAgainAgainAgainAgain,
//
//    }
//
//    PathState pathState;
//    // Mirrored coordinates: redX = 144 - blueX, redHeading = Math.PI - blueHeading
//    private final Pose startPose = new Pose(117.3, 132, Math.toRadians(36));
//    private final Pose shootPose1 = new Pose(98, 97.5, Math.toRadians(50));
//    //private final Pose collect1thingstart = new Pose(88, 59, Math.toRadians(0));
//    private final Pose collect1thing = new Pose(125, 60, Math.toRadians(0));
//    private final Pose goToCollect1ControlPoint = new Pose(79, 58.5, Math.toRadians(0));
//    private final Pose shootPose2 = new Pose( 98, 97.5, Math.toRadians(48.5));
//
//    // Control points for shoot2 path
//    private final Pose shoot2ControlPoint1 = new Pose(93, 76, Math.toRadians(0));
//    private final Pose gateCollect1 = new Pose( 128.5, 62, Math.toRadians(25));
//    //private final Pose inBetween1 = new Pose(100, 62, Math.toRadians(22.5));
//    private final Pose shootPose2ToGateControlPoint = new Pose(94, 55.801430517711175, Math.toRadians(0));
//    private final Pose shootBall3 = new Pose(98, 97.5, Math.toRadians(50));
//    private final Pose inBetween2 = new Pose(100, 62, Math.toRadians(22.5));
//    private final Pose gateCollect2 = new Pose( 128.625, 62, Math.toRadians(25));
//    private final Pose shootBall4 = new Pose(89, 88, Math.toRadians(50));
//    private final Pose gateCollect3 = new Pose( 128.75, 62, Math.toRadians(25));
//    private final Pose shootBall5 = new Pose(89, 88, Math.toRadians(50));
//
//    //private final Pose collect3start=new Pose(87, 86, Math.toRadians(0));
//    private final Pose shoot4ToCollect3ControlPoint = new Pose(102.74659400544961, 82.36784741144412, Math.toRadians(0));
//
//    //
//    private final Pose collect3end = new Pose(120, 86, Math.toRadians(0));
//    private final Pose shootBall6 = new Pose(95, 115, Math.toRadians(33));
//
//    private final Pose park = new Pose(103, 84, Math.toRadians(46));
//
//
//
//    private PathChain shoot1, goToCollect1, collect1, shoot2, GateCollect3, shoot6, InBetween1, InBetween2, GateCollect1, GateCollect2, shoot3, awayfromGate, goToCollect3, collect3, shoot4, goToGate, openGate, goToCollect4, collect4, shoot5, parking;
//
//    public void buildPaths() {
//        shoot1 = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, shootPose1))
//                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
//                .build();
//
//        collect1 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose1, goToCollect1ControlPoint, collect1thing))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        shoot2 = follower.pathBuilder()
//                .addPath(new BezierCurve(collect1thing, shoot2ControlPoint1, shootPose2))
//                .setLinearHeadingInterpolation(collect1thing.getHeading(), shootPose2.getHeading())
//                .build();
//        GateCollect1 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose2, shootPose2ToGateControlPoint, gateCollect1))
//                .setLinearHeadingInterpolation(shootPose2.getHeading(), gateCollect1.getHeading())
//                .build();
//
//        shoot3 = follower.pathBuilder()
//                .addPath(new BezierCurve(gateCollect1, shootPose2ToGateControlPoint, shootBall3))
//                .setLinearHeadingInterpolation(gateCollect1.getHeading(), shootBall3.getHeading())
//                .build();
//        GateCollect2 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootBall3, shootPose2ToGateControlPoint, gateCollect2))
//                .setLinearHeadingInterpolation(shootBall3.getHeading(), gateCollect2.getHeading())
//                .build();
//        shoot4 = follower.pathBuilder()
//                .addPath(new BezierCurve(gateCollect2, shootPose2ToGateControlPoint, shootBall4))
//                .setLinearHeadingInterpolation(gateCollect2.getHeading(), shootBall4.getHeading())
//                .build();
//        GateCollect3 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootBall4, shootPose2ToGateControlPoint, gateCollect3))
//                .setLinearHeadingInterpolation(shootBall4.getHeading(), gateCollect3.getHeading())
//                .build();
//        shoot5 = follower.pathBuilder()
//                .addPath(new BezierCurve(gateCollect3, shootPose2ToGateControlPoint, shootBall5))
//                .setLinearHeadingInterpolation(gateCollect3.getHeading(), shootBall5.getHeading())
//                .build();
//
//        collect3 = follower.pathBuilder()
//                .addPath(new BezierCurve(shootBall5, shoot4ToCollect3ControlPoint, collect3end))
//                .setLinearHeadingInterpolation(shootBall5.getHeading(), collect3end.getHeading())
//                .build();
//
//        shoot6 = follower.pathBuilder()
//                .addPath(new BezierLine(collect3end, shootBall6))
//                .setLinearHeadingInterpolation(collect3end.getHeading(), shootBall6.getHeading())
//                .build();
//    }
//
//    public void statePathUpdate() {
//        switch (pathState) {
//            case start:
//                theWheelOfTheOx.setPower(0);
//                launcher.setVelocity(1100);
//                tree.setPower(1);
//                blocker.setPosition(0);
//                launcher.setVelocity(1100); //1725
//                hood.setPosition(1); //0.285
//                follower.setMaxPower(NORMAL_DRIVE_POWER);
//                follower.followPath(shoot1);
//                if (pathTimer.getElapsedTimeSeconds()>0.5) {
//                    theWheelOfTheOx.setPower(-1);
//                }
//                rotator.setTargetPosition(rotatorStartPosition);
//                setPathState(red18closenewbot.PathState.actuallyshoot1);
//                break;
//            case actuallyshoot1:
//                rotator.setTargetPosition(rotatorStartPosition);
//                launcher.setVelocity(1100);
//                if(pathTimer.getElapsedTimeSeconds()>1.9)
//                {
//                    blocker.setPosition(1);
//                }
//                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2.2){
//                    tree.setPower(1);
//                    launcher.setVelocity(1100);
//                    hood.setPosition(1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    theWheelOfTheOx.setPower(-1);
//                    if(pathTimer.getElapsedTimeSeconds()>2.2)
//                    {
//                        launcher.setVelocity(1100);
//                    }
//                    if (pathTimer.getElapsedTimeSeconds()>2.7) {
//                        setPathState(red18closenewbot.PathState.collection);
//                    }
//                }
//                break;
//
//
//            case collection:
//                rotator.setTargetPosition(rotatorStartPosition);
//                hood.setPosition(1);
//                blocker.setPosition(0);
//                theWheelOfTheOx.setPower(0);
//                tree.setPower(1);
//                rotator.setTargetPosition(rotatorStartPosition);
//                if (!follower.isBusy() && !collectionStarted) {
//                    follower.setMaxPower(NORMAL_DRIVE_POWER);
//                    launcher.setVelocity(1100);
//                    hood.setPosition(1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    tree.setPower(1);
//                    follower.followPath(collect1);
//                    collectionStarted = true;
//                    if (pathTimer.getElapsedTimeSeconds()>1.25)
//                    {
//                        theWheelOfTheOx.setPower(-1);
//                    }
//                }
//                if (!follower.isBusy() && collectionStarted) {
//                    setPathState((red18closenewbot.PathState.shoot));
//                }
//                break;
//            case shoot:
//                rotator.setTargetPosition(rotatorStartPosition);
//                if (!follower.isBusy() && !shoot2Started) {
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    hood.setPosition(1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    follower.followPath(shoot2);
//                    launcher.setVelocity(1100);
//                    follower.setMaxPower(NORMAL_DRIVE_POWER);
//                    tree.setPower(1);
//                    shoot2Started = true;
//                }
//                if (!follower.isBusy() && shoot2Started) {
//                    if (pathTimer.getElapsedTimeSeconds()>2)
//                    {
//                        blocker.setPosition(1);
//                    }
//                    if(pathTimer.getElapsedTimeSeconds()>2.25) {
//                        tree.setPower(1);
//                        theWheelOfTheOx.setPower(-1);
//                    }
//                    if(pathTimer.getElapsedTimeSeconds()>2.95) {
//                        setPathState((red18closenewbot.PathState.GateCollection));
//                    }
//                }
//                break;
//
//            case GateCollection:
//                rotator.setTargetPosition(rotatorStartPosition);
//                blocker.setPosition(0);
//                theWheelOfTheOx.setPower(0);
//                if (!follower.isBusy() && !gateCollectionStarted) {
//                    follower.followPath(GateCollect1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    launcher.setVelocity(1120);
//                    tree.setPower(1);
//                    hood.setPosition(1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    gateCollectionStarted = true;
//                }
//                if (pathTimer.getElapsedTimeSeconds()>2.25) {
//                    theWheelOfTheOx.setPower(-1);
//                }
//                if (!follower.isBusy() && gateCollectionStarted && pathTimer.getElapsedTimeSeconds()>3.15) {
//                    setPathState((red18closenewbot.PathState.shootAgain));
//                }
//                break;
//            case shootAgain:
//                rotator.setTargetPosition(rotatorStartPosition);
//                if (!follower.isBusy() && !shoot3Started) {
//                    hood.setPosition(1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    follower.followPath(shoot3);
//                    tree.setPower(1);
//                    follower.setMaxPower(NORMAL_DRIVE_POWER);
//                    shoot3Started = true;
//                }
//                if(pathTimer.getElapsedTimeSeconds()>0.25)
//                {
//                    tree.setPower(0);
//                }
//                if(pathTimer.getElapsedTimeSeconds()>1.85)
//                {
//                    tree.setPower(1);
//                    blocker.setPosition(1);
//                }
//                if (!follower.isBusy() && shoot3Started) {
//                    if(pathTimer.getElapsedTimeSeconds()>1.9) {
//                        theWheelOfTheOx.setPower(-1);
//                    }
//                    if(pathTimer.getElapsedTimeSeconds()>2.75)
//                    {
//                        setPathState((red18closenewbot.PathState.GateCollectionAgain));
//                    }
//                }
//                break;
//            case GateCollectionAgain:
//                if (!follower.isBusy() && !gateCollectionAgainStarted) {
//                    blocker.setPosition(0);
//                    theWheelOfTheOx.setPower(0);
//                    follower.followPath(GateCollect2);
//                    hood.setPosition(1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    launcher.setVelocity(1140);
//                    tree.setPower(1);
//                    hood.setPosition(1);
//                    gateCollectionAgainStarted = true;
//                }
//                if (pathTimer.getElapsedTimeSeconds()>2.25) {
//                    theWheelOfTheOx.setPower(-1);
//                }
//                if (!follower.isBusy() && gateCollectionAgainStarted && pathTimer.getElapsedTimeSeconds()>3.35) {
//                    setPathState((red18closenewbot.PathState.shootAgainAgain));
//                }
//                break;
//            case shootAgainAgain:
//                tree.setPower(0);
//                rotator.setTargetPosition(rotatorStartPosition);
//                if (!follower.isBusy() && !shoot4Started) {
//                    hood.setPosition(1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    follower.followPath(shoot4);
//                    tree.setPower(1);
//                    follower.setMaxPower(NORMAL_DRIVE_POWER);
//                    shoot4Started = true;
//                }
//                if(pathTimer.getElapsedTimeSeconds()>0.25)
//                {
//                    tree.setPower(0);
//                }
//                if (!follower.isBusy() && shoot4Started) {
//                    if(pathTimer.getElapsedTimeSeconds()>1.45)
//                    {
//                        tree.setPower(1);
//                        blocker.setPosition(1);
//                    }
//                    if(pathTimer.getElapsedTimeSeconds()>1.6) {
//                        blocker.setPosition(1);
//                        theWheelOfTheOx.setPower(-1);
//                    }
//                    if(pathTimer.getElapsedTimeSeconds()>2.75)
//                    {
//                        setPathState((red18closenewbot.PathState.GateCollectionAgainAgain));
//                    }
//                }
//                break;
//            case GateCollectionAgainAgain:
//                blocker.setPosition(0);
//                theWheelOfTheOx.setPower(0);
//                if (!follower.isBusy() && !gateCollectionAgainStarted) {
//                    follower.followPath(GateCollect3);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    launcher.setVelocity(1140);
//                    tree.setPower(1);
//                    hood.setPosition(1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    gateCollectionAgainStarted = true;
//                }
//                if (pathTimer.getElapsedTimeSeconds()>2.25) {
//                    theWheelOfTheOx.setPower(-1);
//                }
//                if (!follower.isBusy() && gateCollectionAgainStarted && pathTimer.getElapsedTimeSeconds()>3.35) {
//                    setPathState((red18closenewbot.PathState.shootAgainAgainAgain));
//                }
//                break;
//            case shootAgainAgainAgain:
//                tree.setPower(0);
//                rotator.setTargetPosition(rotatorStartPosition);
//                if (!follower.isBusy() && !shoot4Started) {
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    follower.followPath(shoot5);
//                    follower.setMaxPower(NORMAL_DRIVE_POWER);
//                    tree.setPower(1);
//                    shoot4Started = true;
//                }
//                if(pathTimer.getElapsedTimeSeconds()>0.25)
//                {
//                    tree.setPower(0);
//                }
//                if (!follower.isBusy() && shoot4Started) {
//                    if (pathTimer.getElapsedTimeSeconds()>1.35)
//                    {
//                        tree.setPower(1);
//                        blocker.setPosition(1);
//                    }
//                    if(pathTimer.getElapsedTimeSeconds()>1.5) {
//                        blocker.setPosition(1);
//                        theWheelOfTheOx.setPower(-1);
//                    }
//                    if(pathTimer.getElapsedTimeSeconds()>2.75)
//                    {
//                        setPathState((red18closenewbot.PathState.collectAgainAgainEnd));
//                    }
//                }
//                break;
//
//            case collectAgainAgainEnd:
//                rotator.setTargetPosition(rotatorStartPosition);
//                blocker.setPosition(0);
//                theWheelOfTheOx.setPower(0);
//                if(pathTimer.getElapsedTimeSeconds()>1.5)
//                {
//                    theWheelOfTheOx.setPower(-1);
//                }
//                if (!follower.isBusy() && !collectionStarted) {
//                    follower.setMaxPower(NORMAL_DRIVE_POWER);
//                    launcher.setVelocity(1080);
//                    hood.setPosition(1);
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    tree.setPower(1);
//                    follower.followPath(collect3);
//                    collectionStarted = true;
//                }
//                if (!follower.isBusy() && collectionStarted) {
//                    theWheelOfTheOx.setPower(-1);
//                    if (pathTimer.getElapsedTimeSeconds()>0.25) {
//                        setPathState((red18closenewbot.PathState.shootAgainAgainAgainAgain));
//                    }
//                }
//                break;
//            case shootAgainAgainAgainAgain:
//                rotator.setTargetPosition(rotatorStartPosition);
//                if (!follower.isBusy() && !shoot5Started) {
//                    rotator.setTargetPosition(rotatorStartPosition);
//                    follower.followPath(shoot6);
//                    launcher.setVelocity(1080);
//                    follower.setMaxPower(NORMAL_DRIVE_POWER);
//                    tree.setPower(1);
//                    shoot5Started = true;
//                }
//                if (!follower.isBusy() && shoot5Started) {
//                    if(pathTimer.getElapsedTimeSeconds()>1.5)
//                    {
//                        blocker.setPosition(1);
//                    }
//                    if(pathTimer.getElapsedTimeSeconds()>1.75) {
//                        tree.setPower(1);
//                        theWheelOfTheOx.setPower(-1);
//                    }
//                    if(pathTimer.getElapsedTimeSeconds()>2.95) {
//                        setPathState((red18closenewbot.PathState.done));
//                    }
//                }
//                break;
//            case done:
//                break;
//        }
//    }
//    public void setPathState(PathState newState) {
//        pathState = newState;
//        pathTimer.resetTimer();
//        shoot2Started = false;
//        shoot3Started = false;
//        shoot4Started = false;
//        shoot5Started = false;
//        goAwayFromGateStarted = false;
//        goTowardsGateStarted = false;
//        opengateStarted = false;
//        parkingStarted = false;
//        collectionStarted = false;
//        beginGateCollectionStarted = false;
//        gateCollectionStarted = false;
//        beginGateCollectionAgainStarted = false;
//        gateCollectionAgainStarted = false;
//    }
//
//    @Override
//    public void init() {
//        pathState = PathState.start;
//        blocker = hardwareMap.get(Servo.class, "blocker");
//        blocker.scaleRange(0, 0.362);
//        blocker.setPosition(0);
//        hood = hardwareMap.get(Servo.class, "hood");
//        hood.scaleRange(0,0.0761);
//        hood.setPosition(1);
//        pathTimer = new Timer();
//        opModeTimer = new Timer();
//        follower = ConstantsNewBot.createFollower(hardwareMap);
//        buildPaths();
//        follower.setStartingPose(startPose);
//
//        vision = new Vision(hardwareMap, RedUniversalConstants.PIPELINENUM);
//        telemetry.addLine("Good to go RED");
//        telemetry.update();
//    }
//
//    public void start() {
//        opModeTimer.resetTimer();
//        setPathState(pathState);
//
//        tree = hardwareMap.get(DcMotor.class, "tree");
//        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
//        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
//        hood = hardwareMap.get(Servo.class, "hood");
//        hood.scaleRange(0,0.0761);
//        blocker = hardwareMap.get(Servo.class, "blocker");
//        blocker.scaleRange(0, 0.4);
//
//        rotator = hardwareMap.get(DcMotor.class, "rotator");
//        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rotatorStartPosition=0;
//        rotator.setTargetPosition(rotatorStartPosition);
//        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rotator.setPower(1);
//
//        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        tree.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        double P = 400;
//        double I = 0;
//        double D = 0;
//        double F = 13.2965;
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
//        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        statePathUpdate();
//
//        PoseStorage.savePose(follower.getPose());
//
//        telemetry.addData("paths state", pathState.toString());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("Heading", follower.getPose().getHeading());
//        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
//        telemetry.addData("jolly crusader velocity", launcher.getVelocity());
//    }
//
//    public double getDist(double tyDeg) {
//        double tyRad = Math.abs(Math.toRadians(tyDeg + limelightUpAngle));
//        double dist = y / Math.tan(tyRad);
//        double realDist = 0.55 * dist + 40.3;
//        return realDist;
//    }
//    public double calcVelocity(double dist) {
//        double velocity = 3.30933 * dist + 1507.01002;
//        return velocity;
//    }
//
//    public void intake(double intakePower){
//        tree.setPower(intakePower);
//        if (!gamepad1.right_bumper) {
//            theWheelOfTheOx.setPower(-0.3);
//        }
//    }
//
//    public void adjustRotator(double tx) {
//        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
//        int adjustment = (int) (fracOfSemiCircum * motor180Range);
//        // Red side uses -offset instead of +offset
//        int newPosition = rotator.getCurrentPosition() + adjustment - offset;
//        rotator.setTargetPosition(newPosition);
//    }
//
//    public void adjustHoodBasedOnDistance(double distance) {
//        if (hood != null) {
//            if (distance > DISTANCE_THRESHOLD) {
//                hood.setPosition(FAR_HOOD_POSITION);
//            } else {
//                hood.setPosition(CLOSE_HOOD_POSITION);
//            }
//        }
//    }
//
//    public void updateLimelightAdjustments() {
//        if (limelight != null) {
//            LLResult ll = limelight.getLatestResult();
//            if (ll != null && ll.isValid()) {
//                txDeg = ll.getTx();
//                tyDeg = ll.getTy();
//                lastValidTx = txDeg;
//                lastValidTy = tyDeg;
//                double currentDistance = getDist(tyDeg);
//                if (currentDistance > 0) {
//                    lastValidDistance = currentDistance;
//                    hasValidLimelightData = true;
//                    adjustRotator(txDeg);
//                    launcher.setVelocity(calcVelocity(currentDistance));
//                    adjustHoodBasedOnDistance(currentDistance);
//                }
//            } else {
//                if (hasValidLimelightData) {
//                    adjustRotator(lastValidTx);
//                    if (lastValidDistance > 0) {
//                        launcher.setVelocity(calcVelocity(lastValidDistance));
//                        adjustHoodBasedOnDistance(lastValidDistance);
//                    }
//                }
//            }
//        }
//    }
//
//    public void resetLimelightData() {
//        hasValidLimelightData = false;
//        lastValidTx = 0.0;
//        lastValidTy = 0.0;
//        lastValidDistance = 0.0;
//    }
//}
