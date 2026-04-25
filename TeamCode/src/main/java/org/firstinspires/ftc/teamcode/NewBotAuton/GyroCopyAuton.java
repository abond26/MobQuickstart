package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.RedUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TestTurret;

import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;

@Autonomous(name = "GyroCopyAuton")
public class GyroCopyAuton extends OpMode implements BlueUniversalConstants {

    Robot robot;
    RobotActions actions;
    TestTurret testTurret;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private boolean shooting = false;
    double targetRPM;
    Pose activeTarget = new Pose(2, 144, 0);
    double ENCODER_CPM  = 28;      // counts per motor revolution
    double GEAR_RATIO   = 1.57;
    private int gateCycleCount = 0;





    // ===================== STATE MACHINE =====================
    public enum PathState{
        StartToShoot,
        Bang1,
        GetReadyForIntake,
        Intake2,
        Goback,
        Bang2,
        GoingToTheFreakingGate,
        IntakingAtGate,
        GateToShoot,
        Bang3,
        Done



    }

    PathState pathState;

    private final Pose startPose = new Pose(26.345407503234163, 127.89974126778786, Math.toRadians(142));
    private final Pose shootPose1 = new Pose(56.73221216041397, 82.2, Math.toRadians(142));
    private final Pose GoToIntakeSet2 = new Pose(43.48273610548573, 57,Math.toRadians(180));
    private final Pose IntakeSet2 = new Pose(11.965683195946655, 57, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(66.7, 73.13906856403621, Math.toRadians(180));
    private final Pose gateCollect1 = new Pose(10.900242948285394, 60.19105419358128, Math.toRadians(148));
    private final Pose GoToIntakeSet1 = new Pose(43.48273610548573, 82.2, Math.toRadians(180));
    private final Pose IntakeSet1 = new Pose(11, 82.2, Math.toRadians(180));







    // ===================== PATHS =====================
    private PathChain StartingToShooting,ShootingToGettingReady2,IntakingSet2,GoingBackToShoot2,ShootToGateIntake,GateIntakeToShoot,ShootTogettingReady1,IntakingSet1,GoingBackToShoot1;



    public void buildPaths(){
        StartingToShooting = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose1.getHeading())
                .build();
        ShootingToGettingReady2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose1, GoToIntakeSet2))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), GoToIntakeSet2.getHeading())
                .build();
        IntakingSet2 = follower.pathBuilder()
                .addPath(new BezierLine(GoToIntakeSet2, IntakeSet2))
                .setLinearHeadingInterpolation(GoToIntakeSet2.getHeading(), IntakeSet2.getHeading())
                .build();
        GoingBackToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeSet2, shootPose2))
                .setLinearHeadingInterpolation(IntakeSet2.getHeading(), shootPose2.getHeading())
                .build();
        ShootToGateIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, gateCollect1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), gateCollect1.getHeading())
                .build();
        GateIntakeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(gateCollect1, shootPose2))
                .setLinearHeadingInterpolation(gateCollect1.getHeading(), shootPose2.getHeading())
                .build();
        ShootTogettingReady1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2, GoToIntakeSet1))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), GoToIntakeSet1.getHeading())
                .build();
        IntakingSet1 = follower.pathBuilder()
                .addPath(new BezierLine(GoToIntakeSet1, IntakeSet1))
                .setLinearHeadingInterpolation(GoToIntakeSet1.getHeading(), IntakeSet1.getHeading())
                .build();
        GoingBackToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeSet1, shootPose2))
                .setLinearHeadingInterpolation(IntakeSet1.getHeading(), shootPose2.getHeading())
                .build();





    }

    // ===================== STATE MACHINE LOGIC =====================
    public void statePathUpdate() {
        switch (pathState) {
            case StartToShoot:
                robot.gate.block();
                targetRPM = 2041.5;
                double ticksPerSec = (targetRPM * ENCODER_CPM * GEAR_RATIO) / 60.0;
                testTurret.setVelocity(ticksPerSec);
                testTurret.setRotatorToAngle(10.9946);
                
                if (follower.atParametricEnd()) {
                    setPathState(PathState.Bang1);
                }
                break;
                
            case Bang1:
                actions.AutonLaunch(1);
                
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    robot.intake.simpleIntake(0);
                    robot.gate.block();
                    
                    follower.followPath(ShootingToGettingReady2, true);
                    setPathState(PathState.GetReadyForIntake);
                }
                break;
            case GetReadyForIntake:
                testTurret.setRotatorToAngle(47.6);
                
                targetRPM = 2471.1;
                testTurret.setVelocity((targetRPM * ENCODER_CPM * GEAR_RATIO) / 60.0);
                
                if (follower.atParametricEnd()) {
                    follower.followPath(IntakingSet2, true);
                    setPathState(PathState.Intake2);
                }
                break;
                
            case Intake2:
                robot.intake.simpleIntake(1);
                
                if (follower.atParametricEnd()) {
                    robot.intake.simpleIntake(0);
                    follower.followPath(GoingBackToShoot2, true);
                    setPathState(PathState.Goback);
                }
                break;

            case Goback:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.Bang2);
                }
                break;
                
            case Bang2:
                actions.AutonLaunch(1);
                
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    robot.intake.simpleIntake(0);
                    robot.gate.block();
                    
                    follower.followPath(ShootToGateIntake, true);
                    setPathState(PathState.GoingToTheFreakingGate);
                }
                break;
                
            case GoingToTheFreakingGate:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.IntakingAtGate);
                }
                break;
                
            case IntakingAtGate:
                robot.intake.simpleIntake(1);
                
                if (pathTimer.getElapsedTimeSeconds() > 2.5 || robot.intake.getDetectedColor(telemetry) == Intake.DetectedColor.BALL) {
                    robot.intake.simpleIntake(0);
                    
                    follower.followPath(GateIntakeToShoot, true);
                    setPathState(PathState.GateToShoot);
                }
                break;
                
            case GateToShoot:
                if (follower.atParametricEnd()) {
                    setPathState(PathState.Bang3);
                }
                break;
                
            case Bang3:
                actions.AutonLaunch(1);
                
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    robot.intake.simpleIntake(0);
                    robot.gate.block();
                    
                    if (gateCycleCount < 4) {
                        gateCycleCount++;
                        follower.followPath(ShootToGateIntake, true);
                        setPathState(PathState.GoingToTheFreakingGate);
                    } else {
                        setPathState(PathState.Done);
                    }
                }
                break;
                case  Done:
                    break;

        }
        
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        shooting = false;
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
        follower = robot.chassisLocal.getFollower();
        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                robot.turret,
                robot.gate,
                robot.intake);

        buildPaths();
        pathState = PathState.StartToShoot;
        pathTimer = new Timer();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(StartingToShooting, true);
        setPathState(PathState.StartToShoot);
    }

    @Override
    public void loop() {
        robot.chassisLocal.update();
        statePathUpdate();

        testTurret.update(robot.chassisLocal.getDistance(activeTarget));



        telemetry.addData("State", pathState);
        telemetry.addData("Path Timer", "%.1f s", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", "%.1f", robot.chassisLocal.getPose().getX());
        telemetry.addData("Y", "%.1f", robot.chassisLocal.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robot.chassisLocal.getPose().getHeading()));
        telemetry.addData("Launcher Velo", robot.turret.getVelocity());
        telemetry.update();
    }
}
