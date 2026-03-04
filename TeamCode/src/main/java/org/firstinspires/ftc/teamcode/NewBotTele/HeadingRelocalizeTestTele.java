package org.firstinspires.ftc.teamcode.NewBotTele;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.HeadingRelocalizeTest.ExperimentalRelocalization;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.HeadingRelocalizeTest.ExperimentalTurret;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.robotControl.Subsystems.test.LimelightRelocalizationConstants;

/**
 * ═══════════════════════════════════════════════════════════════════
 *  EXPERIMENTAL TELEOP — Heading Relocalization & Turret Sync
 * ═══════════════════════════════════════════════════════════════════
 *
 *  This is a full copy of BigBoyBlue with advanced slippage recovery:
 *    TOUCHPAD   = Master Sync (Fixes Gear Slip AND IMU Drift)
 *    DPAD DOWN  = Deadwheel Relocalize (Position Only)
 *
 *  All other controls match BigBoyBlue.
 * ═══════════════════════════════════════════════════════════════════
 */
@TeleOp(name = "HeadingRelocalizeTestTele", group = "test")
public class HeadingRelocalizeTestTele extends LinearOpMode implements BlueUniversalConstants, TurretConstants, LimelightRelocalizationConstants {
    Pose sillyTarget;
    Robot robot;
    RobotActions actions;
    private int veloSwitchNum = 1;
    boolean autoControls = false;
    boolean sillyControls = false;
    private boolean lastDpadUp = false;

    // ── Experimental Components ──
    private ExperimentalRelocalization relocalization;
    private ExperimentalTurret experimentalTurret;
    
    private boolean autoRelocalize = false;
    private boolean lastTouchpad = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = PoseStorage.loadPose(defaultPose);
        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
        
        // Use the Experimental Turret for gear-slip compensation
        experimentalTurret = new ExperimentalTurret(hardwareMap);
        
        relocalization = new ExperimentalRelocalization(robot.vision.getLimelight());

        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                experimentalTurret, // Use experimental version!
                robot.gate,
                robot.intake);

        waitForStart();
        robot.chassisLocal.startTeleop();

        double lastLimelightPollTime = 0;
        com.qualcomm.hardware.limelightvision.LLResult cachedResult = null;

        while (opModeIsActive()) {
            // DRIVING
            robot.chassisLocal.update();
            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            // POLL LIMELIGHT
            double currentTime = getRuntime();
            if (currentTime - lastLimelightPollTime > 0.05) {
                cachedResult = robot.vision.getLimelight().getLatestResult();
                lastLimelightPollTime = currentTime;
            }

            // ═══════════════════════════════════════════════════
            // EXPERIMENTAL SYNC LOGIC
            // ═══════════════════════════════════════════════════

            // 1. DPAD DOWN = Position Only (Deadwheels)
            if (gamepad1.dpad_down && (cachedResult != null)) {
                Pose poseCorrection = relocalization.getRelocalizationPose(
                        robot.chassisLocal.getPose(),
                        cachedResult,
                        false, 
                        experimentalTurret.getRotatorPos()
                );
                if (poseCorrection != null) {
                    robot.chassisLocal.setPose(poseCorrection);
                    gamepad1.rumble(250);
                }
            }

            // 2. TOUCHPAD = Two-Step "Master Sync" (Fixes Slip + Heading)
            if (gamepad1.touchpad && !lastTouchpad && (cachedResult != null)) {
                // Step 1: Fix Turret Encoder using IMU as baseline
                Integer resyncTicks = relocalization.getVisualTurretTicks(
                        Math.toDegrees(robot.chassisLocal.getPose().getHeading()),
                        cachedResult
                );
                
                if (resyncTicks != null) {
                    experimentalTurret.resetRotatorEncoder(resyncTicks);
                    
                    // Step 2: Now fix the Chassis Heading using the calibrated turret
                    Pose finalCorrection = relocalization.getRelocalizationPose(
                            robot.chassisLocal.getPose(), 
                            cachedResult, 
                            true, // Now trustworthy!
                            experimentalTurret.getRotatorPos()
                    );
                    
                    if (finalCorrection != null) {
                        robot.chassisLocal.setPose(finalCorrection);
                        gamepad1.rumble(1000); // Confirmed sync!
                    }
                }
            }
            lastTouchpad = gamepad1.touchpad;

            // ═══════════════════════════════════════════════════
            // ORIGINAL ROBOT CONTROLS (from BigBoyBlue)
            // ═══════════════════════════════════════════════════

            // Turret velocity
            if (gamepad1.aWasPressed()) experimentalTurret.shiftVelocity(launcherSpeedIncrement);
            if (gamepad1.yWasPressed()) experimentalTurret.shiftVelocity(-launcherSpeedIncrement);
            if (gamepad1.rightStickButtonWasPressed()) {
                experimentalTurret.presetVeloSwitch(veloSwitchNum);
                veloSwitchNum = (veloSwitchNum % 4) + 1;
            }

            // Hood control
            actions.hoodControl(gamepad1.x, gamepad1.b);

            // Rotator control (Manual)
            if (gamepad1.dpad_left) {
                experimentalTurret.shiftRotator(-rotatorIncrement);
                sillyControls = false;
            }
            if (gamepad1.dpad_right) {
                experimentalTurret.shiftRotator(rotatorIncrement);
                sillyControls = false;
            }

            // Launch / Intake
            actions.launch(1, gamepad1.right_bumper, sillyTarget);
            if (!gamepad1.right_bumper) {
                actions.intake(gamepad1.left_trigger - gamepad1.right_trigger);
            }

            // Auto-aim toggle
            if (gamepad1.leftStickButtonWasPressed()) sillyControls = !sillyControls;

            // Manual Pose Reset (Dpad Up)
            if (gamepad1.dpad_up && !lastDpadUp) actions.setRobotPose(dpadUpPose);
            lastDpadUp = gamepad1.dpad_up;

            // Run Auto-Aim
            sillyTarget = robot.chassisLocal.sillyTargetPose(target);
            if (sillyControls) {
                actions.aimRotatorLocal(sillyTarget, telemetry);
                actions.adjustShootingParams(sillyTarget);
            }

            // TELEMETRY
            telemetry.addLine("═══ SLIPPAGE RECOVERY (TEST) ═══");
            telemetry.addData("Chassis Heading", "%.1f\u00b0", Math.toDegrees(robot.chassisLocal.getPose().getHeading()));
            telemetry.addData("Turret Physical", "%d (%.1f\u00b0)", 
                    experimentalTurret.getRotatorPos(), 
                    (experimentalTurret.getRotatorPos() - ROTATOR_ZERO_TICKS) / TICKS_PER_DEGREE);
            telemetry.addLine("\u2794 Press TOUCHPAD to Sync Gears + Heading");
            telemetry.update();
        }
        
        // Cleanup Limelight
        try { if (robot.vision.getLimelight() != null) robot.vision.getLimelight().stop(); } catch (Exception e) {}
    }
}
