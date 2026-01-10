package org.firstinspires.ftc.teamcode.Kishen;

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
import org.firstinspires.ftc.teamcode.pedroPathing.WORKPLS;

@Autonomous(name = "HELPER", group = "Examples")
public class HelperAuton extends OpMode {
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;

    private Servo hood;
    private int limeHeight = 33;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
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

    public enum PathState {
        PrepShot1,
        TraeYoung1,
        GoToIntake,
        VerrrryFirstIntake,
        PrepShot2,
        TraeYoung2,
        GoToIntake2,
        HelpIntake1,
        PrepShot3,
        TraeYoung3,
        GoToIntake3,
        HelpIntake2,
        PrepShot4,
        TraeYoung4,
        GoToEnd,

    }

    PathState pathState;
    private final Pose startPose = new Pose(87, 10, Math.toRadians(90));
    private final Pose shootPose = new Pose(87, 12, Math.toRadians(70));





    private final Pose Startofballs3 = new Pose(100, 36, Math.toRadians(0));


    private final Pose EndofBalls3 = new Pose(125, 36, Math.toRadians(0));

    private final Pose HelpyIntake = new Pose(135, 10, Math.toRadians(0));
    private final Pose endPose = new Pose(120, 15, Math.toRadians(0));






    private PathChain GettingReadyToIntake,Intake1,Prep1,HIntake1,Prep2,Endy;

    public void buildPaths() {

        GettingReadyToIntake = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Startofballs3))
                .setLinearHeadingInterpolation(startPose.getHeading(), Startofballs3.getHeading())
                .build();



        Intake1 = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs3, EndofBalls3))
                .setLinearHeadingInterpolation(Startofballs3.getHeading(), EndofBalls3.getHeading())
                .build();


        Prep1 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls3, shootPose))
                .setLinearHeadingInterpolation(EndofBalls3.getHeading(), shootPose.getHeading())
                .build();

        HIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, HelpyIntake ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), HelpyIntake.getHeading())
                .build();

        Prep2 = follower.pathBuilder()
                .addPath(new BezierLine(HelpyIntake, shootPose))
                .setLinearHeadingInterpolation(HelpyIntake.getHeading(), shootPose.getHeading())
                .build();

        Endy = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case PrepShot1:
                if (!follower.isBusy()) {
                    adjustRotator(-3.9);
                    launcher.setVelocity(2100);
                    setPathState(PathState.TraeYoung1);


                }
                break;

            case TraeYoung1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>4) {
                    adjustRotator(-0.001);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);
                    setPathState(PathState.GoToIntake);



                }
                break;

            case GoToIntake:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.5) {
                    follower.followPath(GettingReadyToIntake, true);
                    intake(0);
                    theWheelOfTheOx.setPower(0);
                    setPathState(PathState.VerrrryFirstIntake);


                }
                break;

            case VerrrryFirstIntake:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    adjustRotator(+3.9);
                    follower.followPath(Intake1, true);
                    tree.setPower(1);
                    setPathState(PathState.PrepShot2);
                }
                break;

            case PrepShot2:
                if (!follower.isBusy()) {
                    adjustRotator(-0.01);
                    follower.followPath(Prep1, true);

                    launcher.setVelocity(2100);
                    setPathState(PathState.TraeYoung2);
                }
                break;

            case TraeYoung2:

                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    adjustRotator(-0.01);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);
                    setPathState(PathState.GoToIntake2);
                }
                break;

            case GoToIntake2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.5) {
                    adjustRotator(-0.01);
                    follower.followPath(HIntake1, true);

                    intake(1);
                    setPathState(PathState.PrepShot3);

                }
                break;

            case PrepShot3:
                if (!follower.isBusy()) {
                    adjustRotator(-0.01);
                    follower.followPath(Prep2, true);

                    launcher.setVelocity(2100);
                    setPathState(PathState.TraeYoung3);
                }
                break;

            case TraeYoung3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    adjustRotator(-0.01);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);
                    setPathState(PathState.GoToIntake3);
                }
                break;

            case GoToIntake3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1.5) {
                    adjustRotator(-0.01);
                    follower.followPath(HIntake1, true);

                    intake(1);
                    setPathState(PathState.PrepShot4);

                }
                break;

            case PrepShot4:
                if (!follower.isBusy()) {
                    adjustRotator(-0.01);
                    follower.followPath(Prep2, true);

                    launcher.setVelocity(2100);
                    setPathState(PathState.TraeYoung4);
                }
                break;

            case TraeYoung4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    adjustRotator(-0.01);
                    intake(1);
                    theWheelOfTheOx.setPower(-1);
                    setPathState(PathState.GoToEnd);
                }
                break;

            case GoToEnd:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2) {
                    adjustRotator(-0.01);
                    follower.followPath(Endy);
                    intake(1);

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
        pathState = PathState.PrepShot1;
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
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        adjustRotator(); // Automatically adjust rotator based on limelight tracking
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

    // Automatically adjust rotator based on limelight tracking
    public void adjustRotator() {
        if (limelight != null && rotator != null) {
            LLResult ll = limelight.getLatestResult();
            double currentTxDeg = 0.0;
            double currentTyDeg = 0.0;
            double ta = 0.0;
            boolean llValid = false;

            if (ll != null) {
                currentTxDeg = ll.getTx();
                currentTyDeg = ll.getTy();
                ta = ll.getTa();
                llValid = ll.isValid();
            }

            // Update class variables for use in other methods
            txDeg = currentTxDeg;
            tyDeg = currentTyDeg;

            if (llValid) {
                telemetry.addData("Ta", ta);
                telemetry.addData("tx", currentTxDeg);
                telemetry.addData("ty", currentTyDeg);

                // Adjust rotator based on limelight tx degrees
                double fracOfSemiCircum = Math.toRadians(currentTxDeg) / Math.PI;
                int adjustment = (int) (fracOfSemiCircum * motor180Range);
                int newPosition = rotator.getCurrentPosition() + adjustment - 28;
                rotator.setTargetPosition(newPosition);
            }
        }
    }

    // Manual adjustment method (for backward compatibility with existing code)
    public void adjustRotator(double tx) {
        if (rotator != null) {
            double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
            int adjustment = (int) (fracOfSemiCircum * motor180Range);
            int newPosition = rotator.getCurrentPosition() + adjustment - 28;
            rotator.setTargetPosition(newPosition);
        }
    }


}