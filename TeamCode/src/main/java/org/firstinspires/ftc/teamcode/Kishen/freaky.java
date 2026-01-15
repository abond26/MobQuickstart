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

@Autonomous(name = "lemmetrysomething", group = "Examples")
public class freaky extends OpMode {
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;

    private Servo hood;
    private int limeHeight = 33;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double PRECISE_DRIVE = 0.2;

    private static final double INTAKE_DRIVE_POWER = 0.8; // tune this

    private int y = tagHeight - limeHeight;
    //Rotator var
    int motor180Range = 910;
    int limelightUpAngle = 25;
    private int vMultiplier = 9;
    private Limelight3A limelight;
    private int rotatorStartPosition = 0; // Store starting position

    private DcMotor leftFront, leftRear, rightFront, rightRear;


    private DcMotorEx launcher;
    private DcMotor tree, theWheelOfTheOx, rotator;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_START_SHOOT,
        PRE_SHOOT,
        GoToGetReady,
        CurvyIntake,
        Clear,
        Shot2Before,
        SHOT2,

    }

    PathState pathState;
    private final Pose startPose = new Pose(27 , 129, Math.toRadians(138));
    private final Pose shootPose = new Pose(50, 105, Math.toRadians(138));


    private final Pose GettingReadyForIntake1 = new Pose(50, 57, Math.toRadians(180));
    private final Pose Intake1Endpoint = new Pose(33, 68, Math.toRadians(180));
    private final Pose Intake1ControlPoint = new Pose(0, 60, Math.toRadians(180));
    private final Pose ClearEnd = new Pose(17, 68, Math.toRadians(180));
    private final Pose Shot2ControlPoint = new Pose(72, 72, Math.toRadians(138));













    private PathChain StartShoot,PrepIntake1,Intake1Curve,Clearing,Shot2Curve;

    public void buildPaths() {
        StartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        PrepIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, GettingReadyForIntake1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), GettingReadyForIntake1.getHeading())
                .build();
        Intake1Curve = follower.pathBuilder()
                .addPath(new BezierCurve(GettingReadyForIntake1,Intake1ControlPoint,Intake1Endpoint))
                .setLinearHeadingInterpolation(GettingReadyForIntake1.getHeading(),Intake1Endpoint.getHeading())
                .build();
        Clearing = follower.pathBuilder()
                .addPath(new BezierLine(Intake1Endpoint, ClearEnd))
                .setLinearHeadingInterpolation(Intake1Endpoint.getHeading(), ClearEnd.getHeading())
                .build();
        Shot2Curve = follower.pathBuilder()
                .addPath(new BezierCurve(ClearEnd,Shot2ControlPoint,shootPose))
                .setLinearHeadingInterpolation(ClearEnd.getHeading(),shootPose.getHeading())
                .build();







    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_SHOOT:
                hood.setPosition(.2);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(StartShoot, true);
                launcher.setVelocity(1670);
                setPathState(PathState.PRE_SHOOT);
                break;


            case PRE_SHOOT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    hood.setPosition(.2);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    launcher.setVelocity(1670);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);

                    setPathState(PathState.GoToGetReady);
                }
                break;
            case GoToGetReady:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    theWheelOfTheOx.setPower(0);
                    intake(0);
                    follower.followPath(PrepIntake1, true);
                    setPathState(PathState.CurvyIntake);
                }
                break;
            case CurvyIntake:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(Intake1Curve, true);
                    intake(0.75);
                    setPathState(PathState.Clear);
                }
                break;
            case Clear:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.setMaxPower(PRECISE_DRIVE);
                    intake(0);
                    follower.followPath(Clearing);
                    setPathState(PathState.Shot2Before);
                }
                break;
            case Shot2Before:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    intake(0);
                    follower.followPath(Shot2Curve);
                    setPathState(PathState.SHOT2);
                }
                break;
            case SHOT2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    hood.setPosition(.2);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    launcher.setVelocity(1670);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);

                }
                break;









        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_START_SHOOT;
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
        hood.scaleRange(0,0.0328);


        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorStartPosition = 0; // Store the starting position
        rotator.setTargetPosition(rotatorStartPosition);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator.setPower(1);



        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }
    }

    @Override
    public void loop() {
        // Continuously keep rotator at starting position (zero)
        rotator.setTargetPosition(rotatorStartPosition);
        rotator.setPower(1); // Ensure rotator has power to move to target position
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
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            txDeg = 0.0; //horizontal deg
            tyDeg = 0.0; //vertical deg
            double ta = 0.0;
            boolean llValid = false;
            if (ll != null) {
                txDeg = ll.getTx();
                tyDeg = ll.getTy();
                ta = ll.getTa();
                llValid = ll.isValid();
            }


            if (llValid) {
                telemetry.addData("Ta", ta);
                telemetry.addData("tx", txDeg);
                telemetry.addData("ty", tyDeg);
            }
        }
        double tyRad = Math.toRadians(tyDeg+limelightUpAngle);
        double dist = y / Math.tan(tyRad);
        return dist;
    }
    public double calcVelocity(double dist) {
        double rice = dist/654.83484;
        double velocity = 949.3757*Math.pow(2.72,rice)+ 83.23996;
        double rpower = velocity/2580;
        return rpower;
    }

    public void intake(double intakePower){
        tree.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-0.3);
        }
    }

}