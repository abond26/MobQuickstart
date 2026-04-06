package org.firstinspires.ftc.teamcode.NewBotAuton;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OldAuto.red18closenewbot;
import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.RedUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;

@Autonomous(name = "GyroCopyAuton")
public class GyroCopyAuton extends OpMode implements BlueUniversalConstants {

    Robot robot;
    RobotActions actions;
    private Timer pathTimer, opModeTimer;




    // ===================== STATE MACHINE =====================
    public enum PathState {
        DRIVE_AND_SHOOT,
        WAIT_SHOOT_ARRIVAL,

        DRIVE_TO_INTAKE_2,
        INTAKE_AT_SET_2,
        DRIVE_TO_SHOOT_FROM_INTAKE_2,
        SHOOT_FROM_INTAKE_2,

        // Phase 3: gate zone 1 → shoot
        DRIVE_TO_GATE_ZONE_1,
        WAIT_AT_GATE_1,
        DRIVE_TO_SHOOT_FROM_GATE_1,
        SHOOT_FROM_GATE_1,

        DRIVE_TO_GATE_ZONE_2,
        WAIT_AT_GATE_2,
        DRIVE_TO_SHOOT_FROM_GATE_2,
        SHOOT_FROM_GATE_2,

        DRIVE_TO_GATE_ZONE_3,
        WAIT_AT_GATE_3,
        DRIVE_TO_SHOOT_FROM_GATE_3,
        SHOOT_FROM_GATE_3,

        // Phase 6: gate zone 4 → shoot
        DRIVE_TO_GATE_ZONE_4,
        WAIT_AT_GATE_4,
        DRIVE_TO_SHOOT_FROM_GATE_4,
        SHOOT_FROM_GATE_4,

        // Phase 7: intake set 1 → final shoot
        DRIVE_TO_INTAKE_1,
        INTAKE_AT_SET_1,
        DRIVE_TO_FINAL_SHOOT,
        DRIVE_TO_INTAKE_3,
        INTAKE_AT_SET_3,
        DRIVE_TO_FINAL_SHOOTKLJLKJKLJKLJKLJLKKLLKJ,
        FINAL_SHOOT,
        FINAL_SHOOT_JIZZ,

        DONE
    }

    PathState pathState;

        private final Pose startPose      = new Pose(25.448900388098327, 131.8809831824062, Math.toRadians(234));
    private final Pose shootPose      = new Pose(67, 76, Math.toRadians(140));
    private final Pose shootPose1      = new Pose(67, 76, Math.toRadians(180));

    private final Pose intake2ControlPoint = new Pose(53.0489, 60.85, Math.toRadians(180));

    private final Pose intakeSet2Pose = new Pose(13.697283311772331, 58.20957309184989, Math.toRadians(180)); // Custom "first intake"
    private final Pose gateZonePose   = new Pose(16.7, 58.9, Math.toRadians(143.5));
    private final Pose gateZonePose2   = new Pose(16.7, 59.9, Math.toRadians(139.5));

    private final Pose gateZonePose3   = new Pose(16.7, 60.9, Math.toRadians(137));

    private final Pose intakeSet1Pose = new Pose(19.21862871927555, 83.54592496765846, Math.toRadians(180));  
    private final Pose intake1ControlPoint = new Pose(46.38874514877101, 80.96442432082793, Math.toRadians(180));
    private final Pose intakeSet3Pose = new Pose(13.56274256144889, 35.44113842173352, Math.toRadians(180));
    private final Pose intake3ControlPoint = new Pose(57.44443725743857, 31.086739974126772, Math.toRadians(180));


    // ===================== PATHS =====================
    private PathChain startToShoot;
    private PathChain shootToIntake2, intake2ToShoot;
    private PathChain shootToGate,shootToGate2,shootToGate3, gateToShoot,gateToShoot2,gateToShoot3;
    private PathChain shootToIntake1, intake1ToShoot,shootToIntake3,intake3ToShoot;



    public void buildPaths() {
        startToShoot = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToIntake2 = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootPose, intake2ControlPoint, intakeSet2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeSet2Pose.getHeading())

                .build();

        intake2ToShoot = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierCurve(intakeSet2Pose, intake2ControlPoint, shootPose1))
                .setLinearHeadingInterpolation(intakeSet2Pose.getHeading(), shootPose.getHeading())
                .build();
        shootToGate = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierLine(shootPose1, gateZonePose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), gateZonePose.getHeading())
                .build();
        shootToGate2 = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierLine(shootPose1, gateZonePose2))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), gateZonePose2.getHeading())
                .build();
        shootToGate3 = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierLine(shootPose1, gateZonePose3))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), gateZonePose3.getHeading())
                .build();

        gateToShoot = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierLine(gateZonePose, shootPose1))
                .setLinearHeadingInterpolation(gateZonePose.getHeading(), shootPose1.getHeading())
                .build();
        gateToShoot2 = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierLine(gateZonePose2, shootPose1))
                .setLinearHeadingInterpolation(gateZonePose2.getHeading(), shootPose1.getHeading())
                .build();
        gateToShoot3 = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierLine(gateZonePose3, shootPose1))
                .setLinearHeadingInterpolation(gateZonePose3.getHeading(), shootPose1.getHeading())
                .build();

        shootToIntake1 = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootPose1, intake1ControlPoint, intakeSet1Pose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), intakeSet1Pose.getHeading())
                .build();


        intake1ToShoot = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierCurve(intakeSet1Pose, intake1ControlPoint, shootPose1))
                .setLinearHeadingInterpolation(intakeSet1Pose.getHeading(), shootPose1.getHeading())
                .build();
        shootToIntake3 = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierCurve(shootPose1, intake3ControlPoint, intakeSet3Pose))
                .setLinearHeadingInterpolation(shootPose1.getHeading(), intakeSet3Pose.getHeading())
                .build();


        intake3ToShoot = robot.chassisLocal.getFollower().pathBuilder()
                .addPath(new BezierCurve(intakeSet3Pose, intake3ControlPoint, shootPose))
                .setLinearHeadingInterpolation(intakeSet3Pose.getHeading(), shootPose1.getHeading())
                .build();

    }

    // ===================== STATE MACHINE LOGIC =====================
    public void statePathUpdate() {
        switch (pathState) {

            case DRIVE_AND_SHOOT:
                robot.gate.block();
                actions.intake(-1);
                robot.intake.shift();

                robot.chassisLocal.followPath(startToShoot, true);
                setPathState(PathState.WAIT_SHOOT_ARRIVAL);
                break;

            case WAIT_SHOOT_ARRIVAL:
                if(pathTimer.getElapsedTimeSeconds()>1.8)
                {
                    robot.gate.open();
                    robot.intake.simpleIntake(-1);
                }


                    if (pathTimer.getElapsedTimeSeconds()>2.6) {
                        robot.gate.block();                        
                        setPathState(PathState.DRIVE_TO_INTAKE_2);
                    }
                
                break;

            case DRIVE_TO_INTAKE_2:
                actions.intake(-1);   // start intake early

                robot.chassisLocal.followPath(shootToIntake2, true);
                setPathState(PathState.INTAKE_AT_SET_2);
                break;

            case INTAKE_AT_SET_2:
                if (pathTimer.getElapsedTimeSeconds()>2.4) {
                    actions.intake(0);
                    setPathState(PathState.DRIVE_TO_SHOOT_FROM_INTAKE_2);
                }
                break;

            case DRIVE_TO_SHOOT_FROM_INTAKE_2:
                robot.gate.block();

                robot.chassisLocal.followPath(intake2ToShoot, true);
                setPathState(PathState.SHOOT_FROM_INTAKE_2);
                break;

            case SHOOT_FROM_INTAKE_2:
                if(pathTimer.getElapsedTimeSeconds()>1.6)
                {
                    robot.gate.open();
                    robot.intake.simpleIntake(-1);                }

                    if (pathTimer.getElapsedTimeSeconds()>2.6) {
                        robot.gate.block();
                        setPathState(PathState.DRIVE_TO_GATE_ZONE_1);
                    }
                
                break;

            case DRIVE_TO_GATE_ZONE_1:
                actions.intake(-1);

                robot.chassisLocal.followPath(shootToGate, true);
                setPathState(PathState.WAIT_AT_GATE_1);
                break;

            case WAIT_AT_GATE_1:
                if (!robot.chassisLocal.isBusy()) {
                    actions.intake(-1);

                    Intake.DetectedColor detected = robot.intake.getDetectedColor(telemetry);

                    if (detected != Intake.DetectedColor.UNKNOWN ||pathTimer.getElapsedTime() > 2) {
                        actions.intake(0);
                        setPathState(PathState.DRIVE_TO_SHOOT_FROM_GATE_1);
                    }
                }
                break;

            case DRIVE_TO_SHOOT_FROM_GATE_1:
                robot.gate.block();

                robot.chassisLocal.followPath(gateToShoot, true);
                setPathState(PathState.SHOOT_FROM_GATE_1);
                break;

            case SHOOT_FROM_GATE_1:
                if(pathTimer.getElapsedTimeSeconds()>1.6)
                {
                    robot.gate.open();
                    robot.intake.simpleIntake(-1);                }

                    if (pathTimer.getElapsedTimeSeconds()>2.6) {
                        robot.gate.block();
                        setPathState(PathState.DRIVE_TO_GATE_ZONE_2);
                    }
                
                break;

            case DRIVE_TO_GATE_ZONE_2:
                actions.intake(-1);

                robot.chassisLocal.followPath(shootToGate2, true); // reusing same path to gate
                setPathState(PathState.WAIT_AT_GATE_2);
                break;

            case WAIT_AT_GATE_2:
                if (!robot.chassisLocal.isBusy()) {
                    actions.intake(-1);

                    Intake.DetectedColor detected = robot.intake.getDetectedColor(telemetry);

                    if (detected != Intake.DetectedColor.UNKNOWN ||pathTimer.getElapsedTimeSeconds() > 2) {
                        actions.intake(0);
                        setPathState(PathState.DRIVE_TO_SHOOT_FROM_GATE_2);
                    }
                }
                break;

            case DRIVE_TO_SHOOT_FROM_GATE_2:
                robot.gate.block();

                robot.chassisLocal.followPath(gateToShoot2, true);
                setPathState(PathState.SHOOT_FROM_GATE_2);
                break;

            case SHOOT_FROM_GATE_2:
                if(pathTimer.getElapsedTimeSeconds()>1.6)
                {
                    robot.gate.open();
                    robot.intake.simpleIntake(-1);                }

                    if (pathTimer.getElapsedTimeSeconds()>2.6) {
                        robot.gate.block();
                        setPathState(PathState.DRIVE_TO_GATE_ZONE_3);
                    }
                
                break;

            case DRIVE_TO_GATE_ZONE_3:
                actions.intake(-1);

                robot.chassisLocal.followPath(shootToGate3, true);
                setPathState(PathState.WAIT_AT_GATE_3);
                break;

            case WAIT_AT_GATE_3:
                if (!robot.chassisLocal.isBusy()) {
                    actions.intake(-1);

                    Intake.DetectedColor detected = robot.intake.getDetectedColor(telemetry);

                    if (detected != Intake.DetectedColor.UNKNOWN || pathTimer.getElapsedTimeSeconds() > 3) {
                        actions.intake(0);
                        setPathState(PathState.DRIVE_TO_SHOOT_FROM_GATE_3);
                    }
                }
                break;

            case DRIVE_TO_SHOOT_FROM_GATE_3:
                robot.gate.block();

                robot.chassisLocal.followPath(gateToShoot3, true);
                setPathState(PathState.SHOOT_FROM_GATE_3);
                break;

            case SHOOT_FROM_GATE_3:
                if(pathTimer.getElapsedTimeSeconds()>1.6)
                {
                    robot.gate.open();
                    robot.intake.simpleIntake(-1);                }

                    if (pathTimer.getElapsedTimeSeconds()>2.6) {
                        robot.gate.block();
                        setPathState(PathState.DRIVE_TO_INTAKE_1);
                    }
                
                break;

            case DRIVE_TO_GATE_ZONE_4:
                actions.intake(-1);

                robot.chassisLocal.followPath(shootToGate, true);
                setPathState(PathState.WAIT_AT_GATE_4);
                break;

            case WAIT_AT_GATE_4:
                if (!robot.chassisLocal.isBusy()) {
                    actions.intake(-1);

                    Intake.DetectedColor detected = robot.intake.getDetectedColor(telemetry);

                    if (detected != Intake.DetectedColor.UNKNOWN || pathTimer.getElapsedTimeSeconds() > 3) {
                        actions.intake(0);
                        setPathState(PathState.DRIVE_TO_SHOOT_FROM_GATE_4);
                    }
                }
                break;

            case DRIVE_TO_SHOOT_FROM_GATE_4:
                robot.gate.block();

                robot.chassisLocal.followPath(gateToShoot, true);
                setPathState(PathState.SHOOT_FROM_GATE_4);
                break;

            case SHOOT_FROM_GATE_4:
                if(pathTimer.getElapsedTimeSeconds()>1.6)
                {
                    robot.gate.open();
                    robot.intake.simpleIntake(-1);                }
                if (!robot.chassisLocal.getFollower().isBusy() && pathTimer.getElapsedTimeSeconds()>2.2){

                    if (pathTimer.getElapsedTimeSeconds()>2.6) {
                        robot.gate.block();
                        setPathState(PathState.DRIVE_TO_INTAKE_1);
                    }
                }
                break;

            case DRIVE_TO_INTAKE_1:
                actions.intake(-1);

                robot.chassisLocal.followPath(shootToIntake1, true);
                setPathState(PathState.INTAKE_AT_SET_1);
                break;

            case INTAKE_AT_SET_1:
                if (pathTimer.getElapsedTimeSeconds()>2.4) {
                    actions.intake(1);
                    setPathState(PathState.DRIVE_TO_FINAL_SHOOT);
                }
                break;

            case DRIVE_TO_FINAL_SHOOT:
                robot.gate.block();

                robot.chassisLocal.followPath(intake1ToShoot, true);
                setPathState(PathState.FINAL_SHOOT);
                break;

            case FINAL_SHOOT:
                if(pathTimer.getElapsedTimeSeconds()>1.6)
                {
                    robot.gate.open();
                    robot.intake.simpleIntake(-1);

                }

                    if (pathTimer.getElapsedTimeSeconds()>2.6) {
                        robot.gate.block();                        
                        setPathState(PathState.DRIVE_TO_INTAKE_3);
                    }
                
                break;
            case DRIVE_TO_INTAKE_3:
                actions.intake(-1);

                robot.chassisLocal.followPath(shootToIntake3, true);
                setPathState(PathState.INTAKE_AT_SET_3);
                break;
            case INTAKE_AT_SET_3:
                if (pathTimer.getElapsedTimeSeconds()>2.4) {
                    actions.intake(1);
                    setPathState(PathState.DRIVE_TO_FINAL_SHOOTKLJLKJKLJKLJKLJLKKLLKJ);
                }
                break;
            case DRIVE_TO_FINAL_SHOOTKLJLKJKLJKLJKLJLKKLLKJ:
                robot.gate.block();

                robot.chassisLocal.followPath(intake3ToShoot, true);
                setPathState(PathState.FINAL_SHOOT_JIZZ);
                break;
            case FINAL_SHOOT_JIZZ:
                if(pathTimer.getElapsedTimeSeconds()>1.6)
                {
                    robot.gate.open();
                    robot.intake.simpleIntake(-1);

                }

                if (pathTimer.getElapsedTimeSeconds()>2.6) {
                    robot.gate.block();
                    setPathState(PathState.DONE);
                }

                break;

            // ──────────── DONE ────────────
            case DONE:
                robot.turret.setVelocity(0);
                actions.intake(0);
                robot.turret.setHoodPos(0);
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                robot.turret,
                robot.gate,
                robot.intake);

        buildPaths();
        pathState = PathState.DRIVE_AND_SHOOT;
        pathTimer = new Timer();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(PathState.DRIVE_AND_SHOOT);
    }

    @Override
    public void loop() {
        robot.chassisLocal.update();
        statePathUpdate();

        actions.aimRotatorLocal(target, telemetry);
        actions.adjustShootingParams(BlueUniversalConstants.target);


        telemetry.addData("State", pathState);
        telemetry.addData("Path Timer", "%.1f s", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("X", "%.1f", robot.chassisLocal.getPose().getX());
        telemetry.addData("Y", "%.1f", robot.chassisLocal.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robot.chassisLocal.getPose().getHeading()));
        telemetry.addData("Launcher Velo", robot.turret.getVelocity());
        telemetry.update();
    }
}
