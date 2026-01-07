package org.firstinspires.ftc.teamcode.Kishen;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.WORKPLS;

@Autonomous(name = "Haolin was here", group = "Examples")
public class AutonBlueClose extends OpMode {
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;

    private Servo hood;
    private int limeHeight = 33;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 0.93;
    private static final double INTAKE_DRIVE_POWER = 0.4; // tune this

    private int y = tagHeight - limeHeight;
    //Rotator var
    int motor180Range = 910;
    int limelightUpAngle = 25;
    private int vMultiplier = 9;
    private Limelight3A limelight;

    private DcMotor leftFront, leftRear, rightFront, rightRear;


    private DcMotorEx launcher;
    private DcMotor tree, theWheelOfTheOx, rotator;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        start,
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


        BEFORE_THIRD,

        VERYYYY_THIRD_INTAKE,

        THIRD_SHOT_PREP,

        PAUSE3,

        SHOT_3,

        JACK_OFF,

    }

    PathState pathState;
    private final Pose startPose = new Pose(24.4, 126.7, Math.toRadians(143));
    private final Pose shootPose1 = new Pose(52, 84, Math.toRadians(180));


    private final Pose collect1thing = new Pose(18.5, 84, Math.toRadians(180));
    private final Pose shootPose2 = new Pose( 52, 84, Math.toRadians(180));


    private final Pose collect2Start = new Pose(52, 60, Math.toRadians(180));
    private final Pose collect2End = new Pose(22, 60, Math.toRadians(180));
    private final Pose shootBall3ControlPoint = new Pose(70, 43, Math.toRadians(180));
    private final Pose shootBall3 = new Pose(60, 22, Math.toRadians(180));


    private final Pose collect3Start = new Pose(46, 36, Math.toRadians(180));

    private final Pose collect3End = new Pose(46, 36, Math.toRadians(180));


    private final Pose shootBall4 = new Pose(60, 22, Math.toRadians(180));



    private final Pose collect4start = new Pose(16, 11, Math.toRadians(180));


    private final Pose collect4end = new Pose(10, 11, Math.toRadians(180));

    private final Pose shootBall5 = new Pose(60, 22, Math.toRadians(180));

    private final Pose park = new Pose(16, 11, Math.toRadians(180));




    private PathChain shoot1, collect1, shoot2, goToCollect2, collect2, shoot3, goToCollect3, collect3, shoot4, goToCollect4, collect4, shoot5, parking;

    public void buildPaths() {
        shoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();


        collect1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, collect1thing))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), collect1thing.getHeading())
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


        shoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(collect2End, shootBall3ControlPoint, shootBall3))
                .setLinearHeadingInterpolation(collect2End.getHeading(), shootBall3.getHeading())
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
                .addPath(new BezierLine(shootBall4,collect4start))
                .setLinearHeadingInterpolation(shootBall4.getHeading(), collect4start.getHeading())
                .build();


        collect4 = follower.pathBuilder()
                .addPath(new BezierLine(collect4start, collect4end))
                .setLinearHeadingInterpolation(collect4start.getHeading(), collect4end.getHeading())
                .build();

        shoot5 = follower.pathBuilder()
                .addPath(new BezierLine(collect4end, shootBall5))
                .setLinearHeadingInterpolation(collect4end.getHeading(), shootBall5.getHeading())
                .build();
        parking = follower.pathBuilder()
                .addPath(new BezierLine(shootBall5, park))
                .setLinearHeadingInterpolation(shootBall5.getHeading(), park.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch (pathState) {
            case start:
                follower.followPath(shoot1);
                setPathState(AutonBlueClose.PathState.collection);
                break;
            case collection:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.followPath(collect1);
                    setPathState((AutonBlueClose.PathState.shoot));
                }
                break;
            case shoot:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(shoot2);
                    setPathState((AutonBlueClose.PathState.collectAgain));
                }
                break;
            case collectAgain:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(goToCollect2);
                    setPathState((AutonBlueClose.PathState.collectAgainEnd));
                }
                break;
            case collectAgainEnd:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(collect2);
                    setPathState((AutonBlueClose.PathState.shootAgain));
                }
                break;
            case shootAgain:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(shoot3);
                    setPathState((AutonBlueClose.PathState.collectAgainAgain));
                }
                break;
            case collectAgainAgain:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1){
                    follower.followPath(goToCollect3);
                    setPathState((AutonBlueClose.PathState.collectAgainAgainEnd));
                }
                break;
            case collectAgainAgainEnd:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(collect3);
                    setPathState((AutonBlueClose.PathState.shootAgainAgain));
                }
                break;
            case shootAgainAgain:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(shoot4);
                    setPathState((AutonBlueClose.PathState.collectAgainAgainAgain));
                }
                break;
            case collectAgainAgainAgain:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(goToCollect4);
                    setPathState((AutonBlueClose.PathState.collectAgainAgainAgainEnd));
                }
                break;
            case collectAgainAgainAgainEnd:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(collect4);
                    setPathState((AutonBlueClose.PathState.shootAgainAgainAgain));
                }
                break;
            case shootAgainAgainAgain:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(shoot5);
                    setPathState((AutonBlueClose.PathState.parklol));
                }
                break;
            case parklol:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    follower.followPath(parking);
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


        rotator = hardwareMap.get(DcMotor.class, "rotator");



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
        adjustRotator();
        follower.update();
        statePathUpdate();
        adjustRotator();
        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("jolly crusader velocity", launcher.getVelocity());
    }

    public void adjustRotator() {
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
        double fracOfSemiCircum = Math.toRadians(txDeg) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        int newPosition = rotator.getCurrentPosition() + adjustment;
        rotator.setTargetPosition(newPosition);
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
        double velocity = 949.3757*Math.pow(2.72,rice)+ 83.43996;
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