package org.firstinspires.ftc.teamcode.kiera;



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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="BlueBackAuton")
public class BlueBackAuton extends OpMode {
    private Follower follower;

    private Servo hood;
    private int limeHeight = 33;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 0.75;
    private static final double INTAKE_DRIVE_POWER = 0.5; // tune this

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
    private int rotatorStartPosition = 0; // Store starting position

    public enum PathState {
        LINEUP_FOR_FIRST_SHOT,
        SHOOT_ONE,
        INTAKE_LINEUP_ONE,
        INTAKE_ONE,

        GONNA_RESET,
        RESET,
        CHECKPOINT,
        LINEUP_TO_SHOOT,
        HOLY_SHOT,
        LINEUP_TO_INTAKE,

        INTAKE_BALLS_3,
        ALIGN_SHOOT,
        SHOT_3,
        SNORLAX



    }

    PathState pathState;

    private final Pose startPose = new Pose(61.12653061224489, 8.228571428571431, Math.toRadians(90));
    private final Pose shootBallsOne = new Pose(60.93061224489796, 13.91020408163265, Math.toRadians(110));


    private final Pose lineupWithBallsOne = new Pose(57.99183673469388, 58.5795918367347, Math.toRadians(180));


    private final Pose ballIntakeOne = new Pose(16.518367346938776, 58.5795918367347, Math.toRadians(180));
    private final Pose lineupToReset = new Pose(30.955102040816328, 71.33469387755102, Math.toRadians(180));


    private final Pose resetPose = new Pose(15.281632653061223, 71.33469387755102, Math.toRadians(180));

    private final Pose checkPoint = new Pose(59.95102040816327, 68.57142857142857, Math.toRadians(180));


    private final Pose shootBallsTwo = new Pose(60.93061224489796, 13.714285714285714, Math.toRadians(105));


    private final Pose lineupWithBallsTwo = new Pose(37.42040816326531, 33.81224489795918, Math.toRadians(180));


    private final Pose ballIntakeTwo = new Pose(15.281632653061223, 33.81224489795918, Math.toRadians(180));

    private final Pose alignment = new Pose(57.54792043399638, 29.164556962025312, Math.toRadians(110));

    private final Pose shootBallsThree = new Pose(60.93061224489796, 12.714285714285714, Math.toRadians(105));

    private PathChain lineUpForFirstShot,goingToIntakeOne,afterIntake,goingToReset,reset,checkpointToShoot,lineupToShoot,lineupToIntakeTwo,intakePathTwo,alignUpTwo,Finally;

    //startPose,shootBallsOne, lineupWithBallsOne, ballIntakeOne, lineupToReset, resetPose, checkPoint, shootBallsTwo, lineupWithBallsTwo, ballIntakeTwo, shootBallsThree;

    public void buildPaths() {
        lineUpForFirstShot = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootBallsOne ))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootBallsOne.getHeading())
                .build();


        goingToIntakeOne = follower.pathBuilder()
                .addPath(new BezierLine(shootBallsOne, lineupWithBallsOne))
                .setLinearHeadingInterpolation(shootBallsOne.getHeading(), lineupWithBallsOne.getHeading())
                .build();


        afterIntake = follower.pathBuilder()
                .addPath(new BezierLine(lineupWithBallsOne, ballIntakeOne))
                .setLinearHeadingInterpolation(lineupWithBallsOne.getHeading(), ballIntakeOne.getHeading())
                .build();


        goingToReset = follower.pathBuilder()
                .addPath(new BezierLine(ballIntakeOne, lineupToReset))
                .setLinearHeadingInterpolation(ballIntakeOne.getHeading(), lineupToReset.getHeading())
                .build();

        reset = follower.pathBuilder()
                .addPath(new BezierLine(lineupToReset, resetPose))
                .setLinearHeadingInterpolation(lineupToReset.getHeading(), resetPose.getHeading())
                .build();


        checkpointToShoot = follower.pathBuilder()
                .addPath(new BezierLine(resetPose, checkPoint))
                .setLinearHeadingInterpolation(resetPose.getHeading(), checkPoint.getHeading())
                .build();

        lineupToShoot = follower.pathBuilder()
                .addPath(new BezierLine(checkPoint, shootBallsTwo))
                .setLinearHeadingInterpolation(checkPoint.getHeading(), shootBallsTwo.getHeading())
                .build();


        lineupToIntakeTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootBallsTwo, lineupWithBallsTwo))
                .setLinearHeadingInterpolation(shootBallsTwo.getHeading(), lineupWithBallsTwo.getHeading())
                .build();


        intakePathTwo = follower.pathBuilder()
                .addPath(new BezierLine(lineupWithBallsTwo, ballIntakeTwo))
                .setLinearHeadingInterpolation(lineupWithBallsTwo.getHeading(), ballIntakeTwo.getHeading())
                .build();
        alignUpTwo = follower.pathBuilder()
                .addPath(new BezierLine(ballIntakeTwo, alignment))
                .setLinearHeadingInterpolation(ballIntakeTwo.getHeading(), alignment.getHeading())
                .build();
        Finally= follower.pathBuilder()
                .addPath(new BezierLine(alignment, shootBallsThree))
                .setLinearHeadingInterpolation(alignment.getHeading(), shootBallsThree.getHeading())
                .build();


    }


    // this is where the front auton stops

    //hood position may need to be adjusted
    // powers also may need to be adjusted
    public void statePathUpdate() {
        switch (pathState) {
            case LINEUP_FOR_FIRST_SHOT:


                launcher.setVelocity(2560);

                hood.setPosition(0.0110);

                follower.followPath(lineUpForFirstShot, true);


                follower.setMaxPower(NORMAL_DRIVE_POWER);
                setPathState(PathState.SHOOT_ONE);
                break;

            case SHOOT_ONE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {


                    theWheelOfTheOx.setPower(-1);
                    tree.setPower(1);
                    setPathState(PathState.INTAKE_LINEUP_ONE);


                }
                break;

            case INTAKE_LINEUP_ONE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {

                    follower.followPath(goingToIntakeOne, true);
                    hood.setPosition(0.0139);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);


                    launcher.setVelocity(0);
                    theWheelOfTheOx.setPower(0);
                    tree.setPower(1);

                    setPathState(PathState.INTAKE_ONE);


                }
                break;

            case INTAKE_ONE:
                if (!follower.isBusy()) {

                    // tree.setPower(1);


                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(afterIntake, true);

                    setPathState(PathState.GONNA_RESET);
                }
                break;

            case GONNA_RESET:
                if (!follower.isBusy()) {

                    tree.setPower(0);

                    follower.setMaxPower(NORMAL_DRIVE_POWER);

                    follower.followPath(goingToReset, true);

                    setPathState(PathState.RESET);

                }
                break;

            case RESET:
                if (!follower.isBusy()) {
                    launcher.setVelocity(0);
                    tree.setPower(0);
                    theWheelOfTheOx.setPower(0);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);

                    follower.followPath(reset, true);

                    setPathState(PathState.CHECKPOINT);

                }
                break;

            case CHECKPOINT:
                if (!follower.isBusy()) {
                    launcher.setVelocity(1630);

                    follower.setMaxPower(NORMAL_DRIVE_POWER);

                    follower.followPath(checkpointToShoot, true);

                    setPathState(PathState.LINEUP_TO_SHOOT);


                }
                break;

            case LINEUP_TO_SHOOT:
                if (!follower.isBusy()) {


                    launcher.setVelocity(2520);

                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(lineupToShoot, true);
                    setPathState(PathState.HOLY_SHOT);
                }
                break;

            case HOLY_SHOT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {

                    tree.setPower(1);
                    theWheelOfTheOx.setPower(-1);

                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    setPathState(PathState.LINEUP_TO_INTAKE);
                }
                break;

            case LINEUP_TO_INTAKE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {

                    launcher.setVelocity(0);

                    theWheelOfTheOx.setPower(0);

                    tree.setPower(1);


                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(lineupToIntakeTwo, true);
                    setPathState(PathState.INTAKE_BALLS_3);

                }
                break;

            case INTAKE_BALLS_3:
                if (!follower.isBusy()){

                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(intakePathTwo, true);
                    setPathState(PathState.ALIGN_SHOOT);

                }
                break;
            case ALIGN_SHOOT:

                if(!follower.isBusy()) {
                    tree.setPower(0);
                    launcher.setVelocity(2520);

                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(alignUpTwo, true);
                    setPathState(PathState.SHOT_3);
                }
                break;
            case SHOT_3:
                if(!follower.isBusy()&&pathTimer.getElapsedTime()>3){

                    tree.setPower(1);
                    theWheelOfTheOx.setPower(-1);

                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(Finally, true);
                    setPathState(PathState.SNORLAX);
                }
                break;
            case SNORLAX:
                if(!follower.isBusy()){
                    hood.setPosition(0);
                    launcher.setPower(0);
                    intake(0);
                    theWheelOfTheOx.setPower(0);
                }




        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.LINEUP_FOR_FIRST_SHOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);

//intake
        tree = hardwareMap.get(DcMotor.class, "tree");
        //transfer or something
        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        //self explanatory
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(0.0117);


        rotator = hardwareMap.get(DcMotor.class, "rotator");


        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1.0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            double txDeg = 0.0; //horizontal deg
            double tyDeg = 0.0; //vertical deg
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
                adjustRotator(txDeg);

            }
            adjustRotator(txDeg);

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



    public void adjustRotator(double tx) {
        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        int newPosition = rotator.getCurrentPosition() + adjustment;
        rotator.setTargetPosition(newPosition);
    }

    public double getDist(double tyDeg) {
        double tyRad = Math.toRadians(tyDeg + limelightUpAngle);
        double dist = y / Math.tan(tyRad);
        return dist;
    }

    public double calcVelocity(double dist) {
        double rice = dist / 654.83484;
        double velocity = 949.3757 * Math.pow(2.72, rice) + 83.43996;
        double rpower = velocity / 2580;
        return rpower;
    }

    public void intake(double intakePower) {
        tree.setPower(intakePower);
        // In autonomous, always run the wheel (removed gamepad check)
        theWheelOfTheOx.setPower(-0.3);
    }
}


