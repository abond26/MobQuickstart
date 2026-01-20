package org.firstinspires.ftc.teamcode.pedroPathing;

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
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BetterThanLeftovers", group = "Examples")
public class WORKPLS extends OpMode {
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;

    private Servo hood;
    private int limeHeight = 33;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 1; // tune this

    private int y = tagHeight - limeHeight;
    
    // Hood adjustment constants
    private static final double DISTANCE_THRESHOLD = 180.0;
    private static final double CLOSE_HOOD_POSITION = 0.32; // Hood position for close shots
    private static final double FAR_HOOD_POSITION = 0.54; // Hood position for far shots
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
        BEFORE_FIRST,
        PAUSE,
        VERYYYY_FIRST_INTAKE,


        FIRST_SHOT_PREP,

        SHOT_1,


        BEFORE_SECOND,

        PAUSE2,

        VERYYYY_SECOND_INTAKE,

        SECOND_SHOT_PREPPREP,

        SECOND_SHOT_PREP,
        SHOT_2,


        BEFORE_THIRD,

        VERYYYY_THIRD_INTAKE,

        THIRD_SHOT_PREP,

        PAUSE3,

        SHOT_3,

        JACK_OFF,

    }

    PathState pathState;
    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(57, 85, Math.toRadians(135));


    private final Pose shootPoseover = new Pose(57, 85, Math.toRadians(140));

    private final Pose Startofballs3 = new Pose(50, 35, Math.toRadians(180));
    private final Pose Midofballs3 = new Pose(42, 35, Math.toRadians(180));


    private final Pose EndofBalls3 = new Pose(7, 35, Math.toRadians(180));

    private final Pose Startofballs2 = new Pose(50, 57, Math.toRadians(180));


    private final Pose Midofballs2 = new Pose(42, 57, Math.toRadians(180));



    private final Pose EndofBalls2 = new Pose(9, 57, Math.toRadians(180));


    private final Pose Startofballs1 = new Pose(42, 83, Math.toRadians(180));

    private final Pose Midofballs1 = new Pose(45, 84, Math.toRadians(180));

    private final Pose EndofBalls1 = new Pose(15, 83, Math.toRadians(180));




    private PathChain StartShoot, GoingtoIntake, treeuno, bang1, GoingtoIntake2, treedos, bang2, GoingtoIntake3, treetres, bang3, bang3prep,pizza,pizza2,pizza3,bang2prep;

    public void buildPaths() {
        StartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();


        GoingtoIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, Startofballs3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Startofballs3.getHeading())
                .build();



        pizza = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs3, Midofballs3))
                .setLinearHeadingInterpolation(Startofballs3.getHeading(), Midofballs3.getHeading())
                .build();


        treetres = follower.pathBuilder()
                .addPath(new BezierLine(Midofballs3, EndofBalls3))
                .setLinearHeadingInterpolation(Midofballs3.getHeading(), EndofBalls3.getHeading())
                .build();

        bang3prep = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls3, Startofballs3))
                .setLinearHeadingInterpolation(EndofBalls3.getHeading(), Startofballs3.getHeading())
                .build();


        bang3 = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs3, shootPose ))
                .setLinearHeadingInterpolation(EndofBalls3.getHeading(), shootPose.getHeading())
                .build();

        GoingtoIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, Startofballs2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Startofballs2.getHeading())
                .build();


        pizza2 = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs2, Midofballs2))
                .setLinearHeadingInterpolation(Startofballs2.getHeading(), Midofballs2.getHeading())
                .build();


        treedos = follower.pathBuilder()
                .addPath(new BezierLine(Midofballs2, EndofBalls2))
                .setLinearHeadingInterpolation(Startofballs2.getHeading(), EndofBalls2.getHeading())
                .build();
        bang2prep = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls2,Startofballs2))
                .setLinearHeadingInterpolation(EndofBalls2.getHeading(), Startofballs2.getHeading())
                .build();


        bang2 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls2, shootPoseover))
                .setLinearHeadingInterpolation(EndofBalls2.getHeading(), shootPose.getHeading())
                .build();

        GoingtoIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseover, Startofballs1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Startofballs1.getHeading())
                .build();
        pizza3 = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs1, Midofballs1))
                .setLinearHeadingInterpolation(Startofballs1.getHeading(), Midofballs1.getHeading())
                .build();

        treeuno = follower.pathBuilder()
                .addPath(new BezierLine(Midofballs1, EndofBalls1))
                .setLinearHeadingInterpolation(Midofballs1.getHeading(), EndofBalls1.getHeading())
                .build();

        bang1 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls1, shootPose))
                .setLinearHeadingInterpolation(EndofBalls1.getHeading(), shootPose.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_SHOOT:
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(StartShoot, true);
                setPathState(PathState.PRE_SHOOT);
                break;


            case PRE_SHOOT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);

                    setPathState(PathState.BEFORE_THIRD);
                }
                break;


            case BEFORE_FIRST:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(GoingtoIntake3, true);
                    intake(0);
                    theWheelOfTheOx.setPower(0);


                    setPathState(PathState.PAUSE);

                }
                break;



            case PAUSE:
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(pizza);
                    setPathState(pathState.VERYYYY_FIRST_INTAKE);

                }

            case VERYYYY_FIRST_INTAKE:
                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(treetres, true);
                    intake(1);
                    theWheelOfTheOx.setPower(0.1);




                    setPathState(PathState.FIRST_SHOT_PREP);
                }
                break;



            case FIRST_SHOT_PREP:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(bang3, true);
                    intake(0);
                    theWheelOfTheOx.setPower(0);

                    setPathState(PathState.SHOT_1);
                }
                break;


            case SHOT_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);

                    setPathState(PathState.JACK_OFF);

                }
                break;


            case BEFORE_SECOND:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(GoingtoIntake2, true);
                    intake(0);
                    theWheelOfTheOx.setPower(0);

                    setPathState(PathState.PAUSE2);
                }
                break;

            case PAUSE2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(pizza2);
                    setPathState(pathState.VERYYYY_SECOND_INTAKE);

                }



            case VERYYYY_SECOND_INTAKE:
                if (!follower.isBusy()){
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(treedos, true);
                    intake(1);
                    theWheelOfTheOx.setPower(0);

                    setPathState(PathState.SECOND_SHOT_PREPPREP);

                }
                break;

            case SECOND_SHOT_PREPPREP:
                if (!follower.isBusy()) {
                    intake(0);
                    theWheelOfTheOx.setPower(0);

                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(bang2prep, true);

                    setPathState(PathState.SECOND_SHOT_PREP);
                }


            case SECOND_SHOT_PREP:
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(bang2, true);
                    intake(0);
                    theWheelOfTheOx.setPower(0);

                    setPathState(PathState.SHOT_2);
                }
                break;


            case SHOT_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);

                    setPathState(PathState.BEFORE_FIRST);
                }
                break;


            case BEFORE_THIRD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(GoingtoIntake, true);
                    intake(0);
                    theWheelOfTheOx.setPower(0);

                    setPathState(PathState.VERYYYY_THIRD_INTAKE);
                }
                break;


            case PAUSE3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(pizza3);
                    setPathState(pathState.VERYYYY_THIRD_INTAKE);

                }





            case VERYYYY_THIRD_INTAKE:
                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(treeuno, true);
                    intake(1);
                    theWheelOfTheOx.setPower(0.1);

                    setPathState(PathState.THIRD_SHOT_PREP);
                }
                break;


            case THIRD_SHOT_PREP:
                if (!follower.isBusy()) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(bang1, true);
                    intake(0);
                    theWheelOfTheOx.setPower(0);

                    setPathState(PathState.SHOT_3);
                }
                break;


            case SHOT_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);

                    setPathState(PathState.BEFORE_SECOND);
                }
                break;


            case JACK_OFF:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    hood.setPosition(0);
                    launcher.setPower(0);
                    intake(0);
                    theWheelOfTheOx.setPower(0);

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
        // Limelight adjustments (rotator, velocity, hood)
        double tyDeg = 0.0;
        boolean llValid = false;
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            if (ll != null && ll.isValid()) {
                double txDeg = ll.getTx();
                tyDeg = ll.getTy();
                llValid = true;
                adjustRotator(txDeg);
                
                // Auto-adjust velocity and hood based on distance
                double currentDistance = getDist(tyDeg);
                if (currentDistance > 0) {
                    launcher.setVelocity(calcVelocity(currentDistance));
                    adjustHoodBasedOnDistance(currentDistance);
                }
            }
        }
        
        rotator.setPower(1); // Ensure rotator has power to move to target position
        follower.update();
        statePathUpdate();
        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("jolly crusader velocity", launcher.getVelocity());
        telemetry.addData("rotator pos", rotator.getTargetPosition());
        telemetry.addData("Limelight Valid", llValid);
    }


    public double getDist(double tyDeg) {
        double tyRad = Math.abs(Math.toRadians(tyDeg+limelightUpAngle));
        double dist = y / Math.tan(tyRad);
        double realDist = 0.55*dist+40.3;
        return realDist;
    }
    public double calcVelocity(double dist) {
        // Return velocity in ticks (matching TesterinoRed formula)
        double velocity = 3.30933 * dist + 1507.01002;
        return velocity;
    }

    public void intake(double intakePower){
        tree.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-0.3);
        }
    }

    public void adjustRotator(double tx) {
        double fracOfFullCircum = Math.toRadians(tx) / (Math.PI);
        int adjustment = (int) (fracOfFullCircum * motor180Range);
        int newPosition = rotator.getCurrentPosition() + adjustment - 24;
        rotator.setTargetPosition(newPosition);
    }

    public void adjustHoodBasedOnDistance(double dist) {
        if (hood != null) {
            if (dist > DISTANCE_THRESHOLD) {
                hood.setPosition(FAR_HOOD_POSITION);
            } else {
                hood.setPosition(CLOSE_HOOD_POSITION);
            }
        }
    }

}