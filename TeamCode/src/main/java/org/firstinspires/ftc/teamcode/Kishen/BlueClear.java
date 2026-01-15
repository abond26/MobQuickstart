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

@Autonomous(name = "Test", group = "zzzz")
public class BlueClear extends OpMode {
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;
    private int offset = 28;

    private Servo hood;
    private int limeHeight = 33;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 0.6; // tune this

    private int y = tagHeight - limeHeight;
    //Rotator var
    int motor180Range = 910;
    int limelightUpAngle = 25;
    private int vMultiplier = 9;
    private Limelight3A limelight;
    
    // Distance threshold for hood adjustment (tune this value)
    private static final double DISTANCE_THRESHOLD = 100.0; // Example: change hood when distance > 100 inches
    private static final double CLOSE_HOOD_POSITION = 0.0119; // Hood position for close shots
    private static final double FAR_HOOD_POSITION = 0.150; // Hood position for far shots

    private DcMotor leftFront, leftRear, rightFront, rightRear;


    private DcMotorEx launcher;
    private DcMotor tree, theWheelOfTheOx, rotator;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        DRIVE_START_SHOOT,
        Shoot1,
        GoToIntake1,
        Intake1,
        Clear,
        gettingreadyforshot2,
        Shoot2,
        GoToIntake2,
        Intake2,
        gettingreadyforshot3,
        Shoot3,
        GoToIntake3,
        Intake3,
        gettingreadyforshot4,
        Shoot4Far


    }

    PathState pathState;
    private final Pose startPose = new Pose(22, 123, Math.toRadians(130));
    private final Pose closeShoot = new Pose(50 , 94, Math.toRadians(130));

    private final Pose ReadyForIntake = new Pose( 50, 50, Math.toRadians(180));
    private final Pose IntakecurveEnd = new Pose( 20, 69.47204161248375, Math.toRadians(180));
    private final Pose IntakecurveFunny = new Pose( 23.407022106631988, 60.483745123537055, Math.toRadians(180));

    private final Pose Clearend = new Pose( 16, 69.47204161248375, Math.toRadians(180));

    private final Pose Shot1Funny = new Pose( 44, 46, Math.toRadians(130));

    private final Pose Startofballs3 = new Pose(50, 35, Math.toRadians(180));

    private final Pose EndofBalls3 = new Pose(4.5, 35, Math.toRadians(180));

    private final Pose Startofballs1 = new Pose(42, 83, Math.toRadians(180));

    private final Pose EndofBalls1 = new Pose(11.5, 83, Math.toRadians(180));
    private final Pose SpookyShot = new Pose(56, 8, Math.toRadians(125));

















    private PathChain lebron1,Gettingready1,TrickyIntakeCurve,clear,lebron2,closestintake,gettingready2,lebron3,gettingready3,fartherintake,lebron4;

    public void buildPaths() {
        lebron1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, closeShoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), closeShoot.getHeading())
                .build();
        Gettingready1 = follower.pathBuilder()
                .addPath(new BezierLine(closeShoot, ReadyForIntake))
                .setLinearHeadingInterpolation(closeShoot.getHeading(), ReadyForIntake.getHeading())
                .build();
        TrickyIntakeCurve = follower.pathBuilder()
                .addPath(new BezierCurve(ReadyForIntake,IntakecurveFunny,IntakecurveEnd))
                .setLinearHeadingInterpolation(ReadyForIntake.getHeading(), IntakecurveEnd.getHeading())
                .build();
        clear = follower.pathBuilder()
                .addPath(new BezierLine(IntakecurveEnd, Clearend))
                .setLinearHeadingInterpolation(IntakecurveEnd.getHeading(), Clearend.getHeading())
                .build();
        lebron2 = follower.pathBuilder()
                .addPath(new BezierCurve(Clearend,Shot1Funny,closeShoot))
                .setLinearHeadingInterpolation(Clearend.getHeading(), closeShoot.getHeading())
                .build();
        gettingready2 = follower.pathBuilder()
                .addPath(new BezierLine(closeShoot, Startofballs3))
                .setLinearHeadingInterpolation(closeShoot.getHeading(), Startofballs3.getHeading())
                .build();
        closestintake = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs3, EndofBalls3))
                .setLinearHeadingInterpolation(Startofballs3.getHeading(), EndofBalls3.getHeading())
                .build();
        lebron3 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls3, closeShoot))
                .setLinearHeadingInterpolation(EndofBalls3.getHeading(), closeShoot.getHeading())
                .build();
        gettingready3 = follower.pathBuilder()
                .addPath(new BezierLine(closeShoot, Startofballs1))
                .setLinearHeadingInterpolation(closeShoot.getHeading(), Startofballs1.getHeading())
                .build();
        fartherintake = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs1, EndofBalls1))
                .setLinearHeadingInterpolation(Startofballs1.getHeading(), EndofBalls1.getHeading())
                .build();
        lebron4 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls1, SpookyShot))
                .setLinearHeadingInterpolation(EndofBalls1.getHeading(), SpookyShot.getHeading())
                .build();














    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_SHOOT:
                launcher.setVelocity(1620);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(lebron1);
                setPathState(PathState.Shoot1);
                break;
            case Shoot1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    adjustRotator(37);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);
                    setPathState(PathState.GoToIntake1);
                }
                break;

            case GoToIntake1:

                if (!follower.isBusy()) {
                    intake(0);
                    theWheelOfTheOx.setPower(0);
                    follower.followPath(Gettingready1);

                    setPathState(PathState.Intake1);
                }
                break;
            case Intake1:
                if (!follower.isBusy()) {
                    follower.followPath(TrickyIntakeCurve);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    intake(1);
                    theWheelOfTheOx.setPower(-0.1);
                    setPathState(PathState.Clear);
                }
                break;
            case Clear:
                if (!follower.isBusy()) {
                    theWheelOfTheOx.setPower(0);
                    intake(0);
                    follower.followPath(clear);
                    setPathState((PathState.gettingreadyforshot2));
                }
                break;
            case gettingreadyforshot2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(lebron2);
                    setPathState(PathState.Shoot2);
                }
                break;
            case Shoot2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    adjustRotator(37);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);
                    setPathState(PathState.GoToIntake2);

                }
                break;
            case GoToIntake2:
                if (!follower.isBusy()) {
                    intake(0);
                    theWheelOfTheOx.setPower(0);
                    follower.followPath(gettingready2);
                    setPathState(PathState.Intake2);
                }
                break;
            case Intake2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(closestintake);
                    intake(1);
                    theWheelOfTheOx.setPower(-0.1);
                    setPathState(PathState.gettingreadyforshot3);
                }
                break;
            case gettingreadyforshot3:
                if (!follower.isBusy()) {
                    intake(0);
                    theWheelOfTheOx.setPower(0);

                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(lebron3);
                    setPathState(PathState.Shoot3);


                }
                break;
            case Shoot3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.5) {
                    adjustRotator(37);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);
                    setPathState(PathState.GoToIntake3);

                }
                break;
            case GoToIntake3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    intake(0);
                    theWheelOfTheOx.setPower(0);
                    follower.followPath(gettingready3);
                    setPathState(PathState.Intake3);
                }
                break;
            case Intake3:
                if (!follower.isBusy()) {
                    intake(1);
                    theWheelOfTheOx.setPower(-0.1);
                    follower.setMaxPower(INTAKE_DRIVE_POWER);
                    follower.followPath(fartherintake);
                    intake(1);
                    setPathState(PathState.Shoot4Far);


                    }

                break;
            case gettingreadyforshot4:
                if (!follower.isBusy()) {
                    launcher.setVelocity(2000);
                    intake(0);
                    theWheelOfTheOx.setPower(0);
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(lebron4);
                    setPathState(PathState.Shoot4Far);


                }
                break;
            case Shoot4Far:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    adjustRotator(37);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);
                    setPathState(PathState.GoToIntake3);
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
        
        // Adjust hood based on distance if button is pressed
        if (gamepad1.right_stick_button) {
            double distance = getDist();
            if (distance > 0) { // Only if we have a valid distance reading
                launcher.setVelocity(calcVelocity(distance));
                adjustHoodBasedOnDistance();
            }
        }
        
        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("jolly crusader velocity", launcher.getVelocity());
        if (limelight != null) {
            telemetry.addData("Distance", getDist());
        }
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

    public double getDist() {
        double currentTyDeg = 0.0;
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            double ta = 0.0;
            boolean llValid = false;
            if (ll != null) {
                txDeg = ll.getTx();
                currentTyDeg = ll.getTy();
                tyDeg = currentTyDeg;
                ta = ll.getTa();
                llValid = ll.isValid();
            }

            if (llValid) {
                telemetry.addData("Ta", ta);
                telemetry.addData("tx", txDeg);
                telemetry.addData("ty", currentTyDeg);
            }
        }
        double tyRad = Math.toRadians(currentTyDeg + limelightUpAngle);
        double dist = y / Math.tan(tyRad);
        return dist;
    }
    
    // Overloaded method for backward compatibility
    public double getDist(double tyDegParam) {
        // If parameter is provided but invalid, use limelight value
        if (tyDegParam == 0.0 || tyDegParam == 180.0) {
            return getDist();
        }
        double tyRad = Math.toRadians(tyDegParam + limelightUpAngle);
        double dist = y / Math.tan(tyRad);
        return dist;
    }
    
    public void adjustHoodBasedOnDistance() {
        if (limelight != null && hood != null) {
            double distance = getDist();
            if (distance > DISTANCE_THRESHOLD) {
                hood.setPosition(FAR_HOOD_POSITION);
            } else {
                hood.setPosition(CLOSE_HOOD_POSITION);
            }
        }
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
    public void adjustRotator(double tx) {
        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        int newPosition = rotator.getCurrentPosition() + adjustment - offset;
        rotator.setTargetPosition(newPosition);

    }
}