package org.firstinspires.ftc.teamcode.NewBotTele;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.robotControl.Subsystems.test.LimelightRelocalizationConstants;

/**
 * ═══════════════════════════════════════════════════════════════════
 * EXPERIMENTAL TELEOP — Heading Relocalization (Trusted Encoder)
 * ═══════════════════════════════════════════════════════════════════
 *
 * Full copy of BigBoyBlue logic with simplified heading reset:
 * TOUCHPAD = Direct Heading Reset (Vision Yaw - Turret Angle)
 * DPAD DOWN = Deadwheel Relocalize (Position Only)
 *
 * Since the gear does not slip, we use the raw encoder + AprilTag
 * to instantly correct any deadwheel/IMU heading drift.
 * ═══════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "HeadingRelocalizeTestTele", group = "test")
public class HeadingRelocalizeTestTele extends LinearOpMode
        implements BlueUniversalConstants, TurretConstants, LimelightRelocalizationConstants {
    Pose sillyTarget;
    Robot robot;
    RobotActions actions;
    private int veloSwitchNum = 1;
    boolean autoControls = false;
    boolean sillyControls = false;
    private boolean lastDpadUp = false;

    // ── Components ──

    private boolean lastTouchpad = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = PoseStorage.loadPose(defaultPose);
        robot = new Robot(hardwareMap, startPose, PIPELINENUM);

        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                robot.turret,
                robot.gate,
                robot.intake);

        waitForStart();
        robot.chassisLocal.startTeleop();

        while (opModeIsActive()) {
            // DRIVING
            robot.chassisLocal.update();
            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // ═══════════════════════════════════════════════════
            // HEADING RELOCALIZATION LOGIC (Using Subsystems)
            // ═══════════════════════════════════════════════════
            
            // Periodically update Limelight orientation with latest IMU heading
            actions.updateLimelight();

            // 1. DPAD DOWN = Position Only (Deadwheels)
            if (gamepad1.dpad_down) {
                if (actions.relocalizePositionOnly()) {
                    gamepad1.rumble(250);
                }
            }

            // 2. TOUCHPAD = Direct Heading Reset (Vision + Trusted Encoder)
            if (gamepad1.touchpad && !lastTouchpad) {
                // Correct X, Y, AND Heading using the direct formula in RobotActions
                if (actions.relocalizeFull(telemetry)) {
                    gamepad1.rumble(750); // Confirmed sync!
                }
            }
            lastTouchpad = gamepad1.touchpad;

            // ═══════════════════════════════════════════════════
            // ORIGINAL ROBOT CONTROLS (from BigBoyBlue)
            // ═══════════════════════════════════════════════════

            // Turret velocity
            if (gamepad1.aWasPressed())
                robot.turret.shiftVelocity(launcherSpeedIncrement);
            if (gamepad1.yWasPressed())
                robot.turret.shiftVelocity(-launcherSpeedIncrement);
            if (gamepad1.rightStickButtonWasPressed()) {
                robot.turret.presetVeloSwitch(veloSwitchNum);
                veloSwitchNum = (veloSwitchNum % 4) + 1;
            }

            // Hood control
            actions.hoodControl(gamepad1.x, gamepad1.b);

            // Rotator control (Manual)
            if (gamepad1.dpad_left) {
                robot.turret.shiftRotator(-rotatorIncrement);
                sillyControls = false;
            }
            if (gamepad1.dpad_right) {
                robot.turret.shiftRotator(rotatorIncrement);
                sillyControls = false;
            }

            // Launch / Intake
            actions.launch(1, gamepad1.right_bumper, sillyTarget);
            if (!gamepad1.right_bumper) {
                actions.intake(gamepad1.left_trigger - gamepad1.right_trigger);
            }

            // Auto-aim toggle
            if (gamepad1.leftStickButtonWasPressed())
                sillyControls = !sillyControls;

            // Manual Pose Reset (Dpad Up)
            if (gamepad1.dpad_up && !lastDpadUp)
                actions.setRobotPose(dpadUpPose);
            lastDpadUp = gamepad1.dpad_up;

            // Run Auto-Aim
            sillyTarget = robot.chassisLocal.sillyTargetPose(target);
            if (sillyControls) {
                actions.aimRotatorLocal(sillyTarget, telemetry);
                actions.adjustShootingParams(sillyTarget);
            }

            // TELEMETRY
            telemetry.addLine("═══ HEADING RELOCALIZATION (TEST) ═══");
            telemetry.addData("Chassis Heading", "%.1f\u00b0",
                    Math.toDegrees(robot.chassisLocal.getPose().getHeading()));
            telemetry.addData("Turret Angle", "%.1f\u00b0",
                    (robot.turret.getRotatorPos() - ROTATOR_ZERO_TICKS) / TICKS_PER_DEGREE);
            telemetry.addLine("─── RECOVERY ───");
            telemetry.addLine("\u2794 To fix drift: Point turret at tag, then tap TOUCHPAD");
            telemetry.addLine("\u2794 For X/Y only: Tap DPAD DOWN");
            telemetry.update();
        }

        // Cleanup Limelight
        try {
            if (robot.vision.getLimelight() != null)
                robot.vision.getLimelight().stop();
        } catch (Exception e) {
        }
    }
}
