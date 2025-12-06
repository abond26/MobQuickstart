package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {

  private Follower follower;
 private Timer pathTimer, actionTimer, opmodeTimer;

 private DcMotor intake, launcher, flicker, x;


 private int pathState;
 private final Pose startPose = new Pose(60.29648894668401, 6.903640923226263, Math.toRadians(90)); // Start Pose of our robot.
  private final Pose scorePose = new Pose(60.29648894668401, 83.51625487646292, Math.toRadians(138)); // Highest (First Set) of Artifacts from the Spike Mark.
  private final Pose PickupIntake1 = new Pose(18.43462791763407  , 83.51625487646292, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
  private final Pose BeforeIntake2 = new Pose(60.29648894668401  , 59, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.

 private final Pose BeforeIntake3 = new Pose(60.29648894668401  , 35  , Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.


  private final Pose PickupIntake2 = new Pose(18.076673977485832, 35, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

  private PathChain StartShoot, FirstIntake, FirstIntakeShot, GoingtoIntake,SecondIntake;
  private ElapsedTime runtime = new ElapsedTime();



  public void buildPaths() {
    /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
    StartShoot = follower.pathBuilder()
        .addPath(new BezierLine(startPose, scorePose))
        .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
        .build();

    FirstIntake = follower.pathBuilder()
        .addPath(new BezierLine(scorePose, PickupIntake1))
        .setLinearHeadingInterpolation(scorePose.getHeading(), PickupIntake1.getHeading() )
        .build();

    FirstIntakeShot = follower.pathBuilder()
        .addPath(new BezierLine(PickupIntake1,scorePose))
        .setLinearHeadingInterpolation(PickupIntake1.getHeading(), scorePose.getHeading() )
        .build();
    GoingtoIntake = follower.pathBuilder()
        .addPath(new BezierLine(scorePose,BeforeIntake2))
        .setLinearHeadingInterpolation(scorePose.getHeading(), BeforeIntake2.getHeading())
        .build();
    SecondIntake = follower.pathBuilder()
        .addPath(new BezierLine(BeforeIntake2,PickupIntake2))
        .setLinearHeadingInterpolation(BeforeIntake2.getHeading(), PickupIntake2.getHeading() )
        .build();
    






  }

  public void autonomousPathUpdate() {
    switch (pathState) {
      case 0:
          follower.followPath(StartShoot, true);
          launcher.setPower(-0.82);
          setPathState(1);
        break;
      case 1:
        telemetry.addLine("done first");

        /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
        if(!follower.isBusy()) {

        }
            setPathState(2);
            double time = runtime.time();
            double newTime = time;
            while (newTime - time < 4) {
                launcher.setPower(-0.8);
                intake.setPower(-0.57);
                flicker.setPower(0.4);



        }
        break;


      case 2:
        if(!follower.isBusy()) {


            setPathState(3);
        }

            break;


      case 3:
        if(!follower.isBusy()) {

          launcher.setPower(0.7);
            follower.followPath(GoingtoIntake);
            setPathState(4);
// spin-up 1.5s


// shot 1
          intake.setPower(-1);
          flicker.setPower(1);



// shot 2
          intake.setPower(-1);
          flicker.setPower(1);



          intake.setPower(0);
          flicker.setPower(0);

// wait


// final shot
          intake.setPower(-1);
          flicker.setPower(1);
//          while (newTime - time < 5.5){
//            newTime = runtime.time()
//          }
//          intake.setPower(0);
//          flicker.setPower(0);

        }
        break;
      case 4:
        if(!follower.isBusy()) {
          intake.setPower(0);
            follower.followPath(GoingtoIntake);
            setPathState(5);
        }
        break;
      case 5:
        if(!follower.isBusy()) {
        }

        setPathState(5);



        






        }
  }

  /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
          public void setPathState(int pState) {
    pathState = pState;
    pathTimer.resetTimer();
  }
  /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
          @Override
  public void loop() {

    // These loop the movements of the robot, these must be called continuously in order to work
    follower.update();
    autonomousPathUpdate();

    // Feedback to Driver Hub for debugging
    telemetry.addData("path state", pathState);
    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.addData("heading", follower.getPose().getHeading());
    telemetry.update();
  }

  /** This method is called once at the init of the OpMode. **/
          @Override
  public void init() {

    intake = hardwareMap.get(DcMotor.class, "intake");
    intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    launcher = hardwareMap.get(DcMotor.class, "launcher");
    launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    flicker = hardwareMap.get(DcMotor.class, "flicker");
    flicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    pathTimer = new Timer();
    opmodeTimer = new Timer();
    opmodeTimer.resetTimer();


    follower = Constants.createFollower(hardwareMap);
    buildPaths();
    follower.setStartingPose(startPose);

  }

  /** This method is called continuously after Init while waiting for "play". **/
          @Override
  public void init_loop() {}

  /** This method is called once at the start of the OpMode.
    * It runs all the setup actions, including building paths and starting the path system **/
          @Override
  public void start() {
    opmodeTimer.resetTimer();
    setPathState(0);
  }

  /** We do not use this because everything should automatically disable **/
          @Override
  public void stop() {}
}


