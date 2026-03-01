package org.firstinspires.ftc.teamcode.NewBotTele;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.test.LimelightRelocalization;
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
@TeleOp(name = "testTele", group = "test")
public class testTele extends LinearOpMode implements BlueUniversalConstants {
    Pose sillyTarget;
    Robot robot;
    RobotActions actions;

    boolean autoControls = false;
    boolean sillyControls = false;
    private boolean lastDpadUp = false;

    // ── Relocalization ──
    private LimelightRelocalization relocalization;
    private boolean autoRelocalize = false;
    private boolean lastDpadDown = false;
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

        // ── Initialize Limelight relocalization (shares Limelight with Vision) ──
        relocalization = new LimelightRelocalization(robot.vision.getLimelight());

        telemetry.addData("Status", "Initialized");
        telemetry.addData("LL Relocalization", relocalization.isConnected() ? "READY" : "NOT CONNECTED");
        telemetry.addData("Start Pose", "%.1f, %.1f, %.1f°",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.update();

        waitForStart();
        robot.chassisLocal.startTeleop();

        int loopCount = 0;
        while (opModeIsActive()) {
            // ═══════════════════════════════════════════════════
            // DRIVING AND LOCALIZATION UPDATE
            // ═══════════════════════════════════════════════════
            robot.chassisLocal.update();
            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // ═══════════════════════════════════════════════════
            // LIMELIGHT MEGATAG2 RELOCALIZATION
            // ═══════════════════════════════════════════════════

            // Feed combined heading (chassis + turret) to Limelight every loop (required for MegaTag2)
            double chassisHeadingDeg = Math.toDegrees(robot.chassisLocal.getPose().getHeading());
            double turretRelativeDeg = LimelightRelocalization.rotatorTicksToDegrees(robot.turret.getRotatorPos());
            relocalization.updateOrientation(chassisHeadingDeg, turretRelativeDeg);

            // TOUCHPAD: Toggle auto-relocalization
            if (gamepad1.touchpad && !lastTouchpad) {
                autoRelocalize = !autoRelocalize;
            }
            lastTouchpad = gamepad1.touchpad;

            // DPAD DOWN: Force single relocalization
            boolean forceRelocalize = gamepad1.dpad_down && !lastDpadDown;
            lastDpadDown = gamepad1.dpad_down;

            // Attempt relocalization
            if (autoRelocalize || forceRelocalize) {
                Pose currentPose = robot.chassisLocal.getPose();
                Pose correctedPose = relocalization.getRelocalizationPose(currentPose);
                if (correctedPose != null) {
                    robot.chassisLocal.setPose(correctedPose);
                }
            }

            // ═══════════════════════════════════════════════════
            // MANUAL CONTROLS (same as BlueTele)
            // ═══════════════════════════════════════════════════

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

            // Intake
            double sumOfTrigs = gamepad1.left_trigger - gamepad1.right_trigger;
            actions.intake(sumOfTrigs);

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

            // Relocalization status (new)
            telemetry.addLine("═══ RELOCALIZATION ═══");
            telemetry.addData("Auto Relocalize", autoRelocalize ? "ON (Touchpad toggle)" : "OFF (Touchpad toggle)");
            telemetry.addData("Force Single", "Press DPAD DOWN");
            relocalization.addTelemetry(telemetry, robot.chassisLocal.getPose());

            // Limelight data (from BlueTele)
            telemetry.addLine("");
            telemetry.addLine("LIMELIGHT DATA");
            boolean bool = robot.vision.hasTarget();
            if (bool) {
                double llDist = robot.vision.getDistance();
                telemetry.addData("Limelight Dist", llDist);
            }
            telemetry.addData("Limelight?", bool);

            // Position data
            telemetry.addLine("");
            telemetry.addLine("Automatic Telemetry");
            telemetry.addLine("--------------------------");
            telemetry.addData("Distance with Local", robot.chassisLocal.getDistance(target));
            telemetry.addData("X Position", robot.chassisLocal.getPose().getX());
            telemetry.addData("Y Position", robot.chassisLocal.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(robot.chassisLocal.getPose().getHeading()));

            // Hardware data
            telemetry.addLine("");
            telemetry.addLine("Basic Hardware Telemetry");
            telemetry.addLine("--------------------------");
            telemetry.addData("Hood position", robot.turret.getHoodPos());
            telemetry.addData("Rotator position", robot.turret.getRotatorPos());
            telemetry.addData("Rotator angle", "%.1f°",
                    LimelightRelocalization.rotatorTicksToDegrees(robot.turret.getRotatorPos()));
            telemetry.addData("Intake power", robot.intake.getPower());
            telemetry.update();
        }

        // Note: intentionally NOT calling relocalization.stop()
        // as limelight.stop() can hang the OpMode on the FTC SDK
    }
}
