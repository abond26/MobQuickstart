package org.firstinspires.ftc.teamcode.NewBotTele;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControl.RedUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;

/**
 * ═══════════════════════════════════════════════════════════════════
 *  TEST TELEOP — BlueTele + Limelight AprilTag Relocalization
 * ═══════════════════════════════════════════════════════════════════
 *
 *  This is a copy of BlueTele with MegaTag2 AprilTag relocalization
 *  added on top. When the Limelight sees an AprilTag, it corrects
 *  the Pedro Pathing deadwheel pose to fix localization drift.
 *
 *  NEW CONTROLS (in addition to all BlueTele controls):
 *    TOUCHPAD   = Toggle auto-relocalization ON/OFF
 *    DPAD DOWN  = Force single relocalization
 *
 *  All original BlueTele controls still work:
 *    Left stick   = drive
 *    Right stick  = rotate
 *    X / B        = hood control
 *    DPAD L/R     = manual rotator
 *    Triggers     = intake
 *    Left stick button  = toggle silly (dynamic) auto-aim
 *    DPAD UP      = relocalize to dpadUpPose (manual override)
 * ═══════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "SmallManRed", group = "test")
public class SmallManRed extends LinearOpMode implements RedUniversalConstants {
    Pose sillyTarget;
    Robot robot;
    RobotActions actions;
    private int veloSwitchNum = 1;
    boolean autoControls = false;
    boolean sillyControls = false;
    private boolean lastDpadUp = false;

    // ── Relocalization ──
    private boolean autoRelocalize = false;
    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastTouchpad = false;



    // Limelight pipeline for AprilTag 3D (must match your Limelight config)
    private static final int APRILTAG_PIPELINE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = PoseStorage.loadPose(defaultPose);
        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
        robot.chassisLocal.update();

        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                robot.turret,
                robot.gate,
                robot.intake);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        robot.chassisLocal.startTeleop();

        while (opModeIsActive()) {
            // ═══════════════════════════════════════════════════
            // DRIVING AND LOCALIZATION UPDATE
            // ═══════════════════════════════════════════════════
            robot.chassisLocal.update();
            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // ═══════════════════════════════════════════════════
            // LIMELIGHT MEGATAG1 RELOCALIZATION
            // ═══════════════════════════════════════════════════

            actions.updateLimelight();

            if (gamepad1.touchpad && !lastTouchpad) {
                autoRelocalize = !autoRelocalize;
            }
            lastTouchpad = gamepad1.touchpad;

            if (gamepad1.dpad_down) {
                actions.relocalizePositionOnly();
            }

            if (gamepad1.left_bumper) {
                actions.relocalizeFull(telemetry);
            }

            if (autoRelocalize) {
                actions.relocalizePositionOnly();
            }

            // ═══════════════════════════════════════════════════
            // MANUAL CONTROLS — SHOOTING (same as BlueTele)
            // ═══════════════════════════════════════════════════

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
            }

            // Hood control
            actions.hoodControl(gamepad1.x, gamepad1.b);

            // Rotator control
            if (gamepad1.dpad_left) {
                robot.turret.shiftRotator(-rotatorIncrement);
                sillyControls = false;
            }
            if (gamepad1.dpad_right) {
                robot.turret.shiftRotator(rotatorIncrement);
                sillyControls = false;
            }

            // Feed (launch)
            actions.launch(1, gamepad1.right_bumper, sillyTarget);
            if (gamepad1.right_bumper) {
                gamepad1.rumble(100);
            }

            // Intake — only when not launching (same as BlueTele)
            double sumOfTrigs = gamepad1.left_trigger - gamepad1.right_trigger;
            if (!gamepad1.right_bumper) {
                actions.intake(sumOfTrigs);
            }

            // ═══════════════════════════════════════════════════
            // AUTOMATIC CONTROLS (same as BlueTele)
            // ═══════════════════════════════════════════════════

            if (gamepad1.leftStickButtonWasPressed()) {
                sillyControls = !sillyControls;
            }

            // Manual relocalization (DPAD UP — same as BlueTele)
            if (gamepad1.dpad_up && !lastDpadUp) {
                actions.setRobotPose(dpadUpPose);
            }
            lastDpadUp = gamepad1.dpad_up;

            // Dynamic shooting
            sillyTarget = robot.chassisLocal.sillyTargetPose(target);
            if (sillyControls & !autoControls) {
                actions.aimRotatorLocal(sillyTarget, telemetry);
                actions.adjustShootingParams(sillyTarget);
            }

            // ═══════════════════════════════════════════════════
            // TELEMETRY
            // ═══════════════════════════════════════════════════

            telemetry.addData("Auto Relocalize", autoRelocalize);

            // Position data
            telemetry.addLine("");
            telemetry.addLine("ROBOT POSITION");
            telemetry.addData("X", "%.1f", robot.chassisLocal.getPose().getX());
            telemetry.addData("Y", "%.1f", robot.chassisLocal.getPose().getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(robot.chassisLocal.getPose().getHeading()));

            // Hardware data
            telemetry.addLine("");
            telemetry.addLine("HARDWARE STATUS");
            telemetry.addData("Hood", "%.3f", robot.turret.getHoodPos());
            telemetry.addData("Rotator", "%d (%.1f°)",
                    robot.turret.getRotatorPos(),
                    LimelightRelocalization.rotatorTicksToDegrees(robot.turret.getRotatorPos()));
            telemetry.addData("Intake power", "%.2f", robot.intake.getPower());
            telemetry.addData("Launcher velocity", robot.turret.getVelocity());
            telemetry.update();

        }

        try {
            if (robot.vision != null) {
                robot.vision.getLimelight().stop();
            }
        } catch (Exception e) {}
    }
}

