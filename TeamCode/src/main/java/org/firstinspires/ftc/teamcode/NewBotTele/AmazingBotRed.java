package org.firstinspires.ftc.teamcode.NewBotTele;

import com.qualcomm.robotcore.hardware.Servo;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.robotControl.RedUniversalConstants;


@TeleOp(name = "AmazingBotRed")
public class AmazingBotRed extends LinearOpMode implements BlueUniversalConstants {
    long loopTime;
    Pose sillyTarget;
    Robot robot;
    RobotActions actions;
    private int veloSwitchNum = 1;
    boolean sillyControls = false;
    boolean inPosition;
    private boolean lastDpadUp = false;


    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastTouchpad = false;

    private static final int APRILTAG_PIPELINE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Log.w("opmode start: ", "start");
        Pose startPose = PoseStorage.loadPose(defaultPose);
        Log.w("loaded start pose: ", startPose.toString());
        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
        Log.w("robot object inited ", robot.toString());
        robot.chassisLocal.update();
        Log.w("chassisLocal updated: ", robot.chassisLocal.toString());


        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                robot.turret,
                robot.gate,
                robot.intake);


        telemetry.addData("Start Pose", "%.1f, %.1f, %.1f°",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();

        Log.w("current pose: ", startPose.toString());

        waitForStart();
        robot.chassisLocal.startTeleop();

        Log.w("teleop", "started teleop");



        boolean firstRun = true;

        while (opModeIsActive()) {
            // DRIVING AND LOCALIZATION UPDATE
            robot.chassisLocal.update();
            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


            lastDpadDown = gamepad1.dpad_down;
            lastTouchpad = gamepad1.touchpad;
            lastLeftBumper = gamepad1.left_bumper;

            // MANUAL CONTROLS — SHOOTING (same as BlueTele)

            // Turret velocity
            if (gamepad1.aWasPressed()) {
                robot.turret.shiftVelocity(launcherSpeedIncrement);
            }
            if (gamepad1.yWasPressed()) {
                robot.turret.shiftVelocity(-launcherSpeedIncrement);
            }
            if (gamepad1.rightStickButtonWasPressed()) {
                robot.turret.presetVeloSwitch(veloSwitchNum);
                veloSwitchNum += 1;
                sillyControls = false;
            }
            if (gamepad1.dpad_left) {
                robot.turret.shiftRotator(-rotatorIncrement);
                sillyControls = false;
            }
            if (gamepad1.dpad_right) {
                robot.turret.shiftRotator(rotatorIncrement);
                sillyControls = false;
            }

            // Hood control
            if (gamepad1.x) {
                robot.turret.shiftHood(-hoodIncrement);
            }
            if (gamepad1.b) {
                robot.turret.shiftHood(hoodIncrement);
            }

            // Feed (launch)
            actions.launch(1, gamepad1.right_bumper);
            if (gamepad1.right_bumper) {
                gamepad1.rumble(100);
            }

            // Intake — only when not launching (same as BlueTele)
            double sumOfTrigs = gamepad1.left_trigger - gamepad1.right_trigger;
            if (!gamepad1.right_bumper) {
                actions.intake(sumOfTrigs);
            }

            // Color sensor detection — auto-shifts intake when ball detected
            Intake.DetectedColor detectedColor = robot.intake.getDetectedColor(telemetry);
            // AUTOMATIC CONTROLS (same as BlueTele)

            if (gamepad1.leftStickButtonWasPressed()) {
                sillyControls = !sillyControls;
            }

            // Intake Shifted towards the top
            if (gamepad1.dpad_up && !lastDpadUp) {
                robot.intake.down();
            }
            lastDpadUp = gamepad1.dpad_up;
            // Intake Shifted towards the intakePos
            if (gamepad1.left_bumper){
                robot.intake.shift();
            }

            // Dynamic shooting
            sillyTarget = robot.chassisLocal.sillyTargetPose(target);
            inPosition = robot.chassisLocal.isShootingPosition();
            if (sillyControls) {
                actions.aimRotatorLocal(sillyTarget, telemetry);
                actions.adjustShootingParams(sillyTarget);
            }

            telemetry.addLine("");
            telemetry.addLine("ROBOT POSITION");
            telemetry.addData("X", "%.1f", robot.chassisLocal.getPose().getX());
            telemetry.addData("Y", "%.1f", robot.chassisLocal.getPose().getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(robot.chassisLocal.getPose().getHeading()));



            // Hardware data
            telemetry.addLine("");
            telemetry.addLine("HARDWARE STATUS");
            telemetry.addData("Hood", "%.3f", robot.turret.getHoodPos());
            telemetry.addData("Rotator", "%.3f", robot.turret.getRotatorPos());

            telemetry.addData("Intake power", "%.2f", robot.intake.getPower());
            telemetry.addData("Ball Detected", detectedColor);
            telemetry.addData("Launcher velocity", robot.turret.getTargetVelocity());
            telemetry.addData("LauncherL velocity", robot.turret.getVelocityL());
            telemetry.addData("LauncherR velocity", robot.turret.getVelocityR());


            telemetry.addData("Distance", robot.chassisLocal.getDistance(target));
            telemetry.update();
        }
    }
}