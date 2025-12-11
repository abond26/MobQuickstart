package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "BetterThanLeftovers", group = "Examples")
public class WORKPLS extends OpMode {
    private Follower follower;



    private DcMotor intake, flicker, launcher, rotator;
    private Timer pathTimer,opModeTimer;
    public enum PathState {
        DRIVE_START_SHOOT,
        PRE_SHOOT,
        BEFORE_FIRST,
        VERYYYY_FIRST_INTAKE,

        FIRST_SHOT_PREP,
        SHOT_1,


        BEFORE_SECOND,

        VERYYYY_SECOND_INTAKE,


        SECOND_SHOT_PREP,
        SHOT_2,


        BEFORE_THIRD,

        VERYYYY_THIRD_INTAKE,

        THIRD_SHOT_PREP,

        SHOT_3,

        JACK_OFF,

    }
    PathState pathState;
    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(57, 80, Math.toRadians(130));

    private final Pose Startofballs3 = new Pose(57,35, Math.toRadians(180));

    private final Pose EndofBalls3 = new Pose(21,35, Math.toRadians(180));

    private final Pose Startofballs2 = new Pose(45,60, Math.toRadians(180));


    private final Pose EndofBalls2 = new Pose(21,60, Math.toRadians(180));


    private final Pose Startofballs1 = new Pose(45,85, Math.toRadians(180));

    
    private final Pose EndofBalls1 = new Pose(21,85, Math.toRadians(180));





    private PathChain StartShoot,GoingtoIntake,intakeuno,bang1,GoingtoIntake2,intakedos,bang2,GoingtoIntake3,intaketres,bang3;
    public void buildPaths() {
        StartShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();


        GoingtoIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,Startofballs3))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Startofballs3.getHeading())
                .build();


        intaketres = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs3,EndofBalls3))
                .setLinearHeadingInterpolation(Startofballs3.getHeading(), EndofBalls3.getHeading())
                .build();




        bang3 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls3,shootPose))
                .setLinearHeadingInterpolation(EndofBalls3.getHeading(), shootPose.getHeading())
                .build();

        GoingtoIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,Startofballs2))
                .setLinearHeadingInterpolation(shootPose.getHeading(), Startofballs2.getHeading())
                .build();

        intakedos = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs2,EndofBalls2))
                .setLinearHeadingInterpolation(Startofballs2.getHeading(), EndofBalls2.getHeading())
                .build();

        bang2 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls2,shootPose))
                .setLinearHeadingInterpolation(EndofBalls2.getHeading(), shootPose.getHeading())
                .build();

        GoingtoIntake =follower.pathBuilder()
                .addPath(new BezierLine(shootPose,Startofballs1))
                .setLinearHeadingInterpolation(shootPose.getHeading(),Startofballs1.getHeading())
                .build();

        intakeuno = follower.pathBuilder()
                .addPath(new BezierLine(Startofballs1,EndofBalls1))
                .setLinearHeadingInterpolation(Startofballs1.getHeading(), EndofBalls1.getHeading())
                .build();

        bang1 = follower.pathBuilder()
                .addPath(new BezierLine(EndofBalls1,shootPose))
                .setLinearHeadingInterpolation(EndofBalls1.getHeading(), shootPose.getHeading())
                .build();


    }
    public void statePathUpdate() {
        switch(pathState){
            case DRIVE_START_SHOOT:
                follower.followPath(StartShoot, true);
                launcher.setPower(-0.8);
                setPathState(PathState.PRE_SHOOT);
                break;






            case PRE_SHOOT:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    launcher.setPower(-0.8);
                    intake.setPower(1);
                    flicker.setPower(1);
                    setPathState(PathState.BEFORE_FIRST);
                }
                break;




  

            case BEFORE_FIRST:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(GoingtoIntake3, true);
                    launcher.setPower(0);
                    intake.setPower(0);
                    flicker.setPower(0);
                    setPathState(PathState.VERYYYY_FIRST_INTAKE);

                }







                break;
            case VERYYYY_FIRST_INTAKE :
                if (!follower.isBusy()){
                    follower.followPath(intaketres, true);
                    intake.setPower(1);
                    flicker.setPower(0.2);
                    setPathState(PathState.FIRST_SHOT_PREP);
                }



            case FIRST_SHOT_PREP :
                if (!follower.isBusy()){
                    follower.followPath(bang3, true);
                    intake.setPower(0);
                    flicker.setPower(0);
                    launcher.setPower(-0.8);
                    setPathState(PathState.SHOT_1);
                }
                break;






            case SHOT_1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    launcher.setPower(-0.8);
                    intake.setPower(1);
                    flicker.setPower(1);
                    setPathState(PathState.BEFORE_SECOND);

                }
                break;







            case BEFORE_SECOND :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(GoingtoIntake2, true);
                    launcher.setPower(0);
                    intake.setPower(0);
                    flicker.setPower(0);
                    setPathState(PathState.VERYYYY_SECOND_INTAKE);
                }
                break;










            case VERYYYY_SECOND_INTAKE :
                if (!follower.isBusy()) {
                    follower.followPath(intakedos, true);
                    intake.setPower(1);
                    flicker.setPower(0.2);
                    setPathState(PathState.SECOND_SHOT_PREP);

                }
                break;
                
                
                
                
                
            case SECOND_SHOT_PREP :
                if (!follower.isBusy()) {
                    follower.followPath(bang2, true);
                    intake.setPower(0);
                    flicker.setPower(0);
                    launcher.setPower(-0.8);
                    setPathState(PathState.SHOT_2);
                }
                break;
                
                
                
                
                
            case SHOT_2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    launcher.setPower(-0.8);
                    intake.setPower(1);
                    flicker.setPower(1);
                    setPathState(PathState.BEFORE_THIRD);
                }
                break;






            case BEFORE_THIRD :
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    follower.followPath(GoingtoIntake, true);
                    launcher.setPower(0);
                    intake.setPower(0);
                    flicker.setPower(0);
                    setPathState(PathState.VERYYYY_THIRD_INTAKE);
                }
                break;





            case VERYYYY_THIRD_INTAKE :
                if (!follower.isBusy()) {
                    follower.followPath(intakeuno, true);
                    intake.setPower(0.8);
                    flicker.setPower(0.2);
                    setPathState(PathState.THIRD_SHOT_PREP);
                }
                break;






            case THIRD_SHOT_PREP :
                if (!follower.isBusy()) {
                    follower.followPath(bang1, true);
                    intake.setPower(0);
                    flicker.setPower(0);
                    launcher.setPower(0.8);
                    setPathState(PathState.SHOT_3);
                }
                break;







            case SHOT_3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2) {
                    launcher.setPower(-0.8);
                    intake.setPower(1);
                    flicker.setPower(1);
                    setPathState(PathState.JACK_OFF);
                }
                break;




            case JACK_OFF:
                if (!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 1){
                    launcher.setPower(0);
                    intake.setPower(0);
                    flicker.setPower(0);
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


        intake = hardwareMap.get(DcMotor.class, "intake");
        flicker = hardwareMap.get(DcMotor.class, "flicker");
        launcher = hardwareMap.get(DcMotor.class, "launcher");


        rotator = hardwareMap.get(DcMotor.class, "rotator");



        flicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());

    }
}