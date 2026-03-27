//package org.firstinspires.ftc.teamcode.NewBotTele;
//
//import android.util.Log;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
//import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
//import org.firstinspires.ftc.teamcode.util.PoseStorage;
//import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
//
///**
// * ═══════════════════════════════════════════════════════════════════
// * TEST TELEOP — BlueTele + Limelight AprilTag Relocalization
// * ═══════════════════════════════════════════════════════════════════
// *
// * This is a copy of BlueTele with MegaTag2 AprilTag relocalization
// * added on top. When the Limelight sees an AprilTag, it corrects
// * the Pedro Pathing deadwheel pose to fix localization drift.
// *
// * NEW CONTROLS (in addition to all BlueTele controls):
// * DPAD DOWN = Position only (one-time)
// * TOUCHPAD = Position + heading (one-time full snap). No auto-relocalization.
// *
// * All original BlueTele controls still work:
// * Left stick = drive
// * Right stick = rotate
// * X / B = hood control
// * DPAD L/R = manual rotator
// * Triggers = intake
// * Left stick button = toggle silly (dynamic) auto-aim
// * DPAD UP = relocalize to dpadUpPose (manual override)
// * ═══════════════════════════════════════════════════════════════════
// */
//@TeleOp(name = "BigBoyBlue", group = "test")
//public class BigBoyBlue extends LinearOpMode implements BlueUniversalConstants {
//    long loopTime;
//    Pose sillyTarget;
//    Robot robot;
//    RobotActions actions;
//    private int veloSwitchNum = 1;
//    boolean sillyControls = false;
//    boolean inPosition;
//    private boolean lastDpadUp = false;
//
//    // ── Relocalization ──
//    private LimelightRelocalization relocalization;
//    private boolean lastDpadDown = false;
//    private boolean lastLeftBumper = false;
//    private boolean lastTouchpad = false;
//
//    // Limelight pipeline for AprilTag 3D (must match your Limelight config)
//    private static final int APRILTAG_PIPELINE = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Log.w("opmode start: ", "start");
//        Pose startPose = PoseStorage.loadPose(defaultPose);
//        Log.w("loaded start pose: ", startPose.toString());
//        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
//        Log.w("robot object inited ", robot.toString());
//        robot.chassisLocal.update();
//        Log.w("chassisLocal updated: ", robot.chassisLocal.toString());
//
//
//        actions = new RobotActions(
//                robot.chassisLocal,
//                robot.vision,
//                robot.turret,
//                robot.gate,
//                robot.intake);
//
//        // ── Initialize Limelight relocalization (shares Limelight with Vision) ──
//        relocalization = new LimelightRelocalization(robot.vision.getLimelight());
//
//        Log.w("limelight", "limelight recieved");
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("LL Relocalization", relocalization.isConnected() ? "READY" : "NOT CONNECTED");
//        telemetry.addData("Start Pose", "%.1f, %.1f, %.1f°",
//                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
//        telemetry.update();
//
//        Log.w("current pose: ", startPose.toString());
//
//        waitForStart();
//        robot.chassisLocal.startTeleop();
//
//        Log.w("teleop", "started teleop");
//
//
//        int loopCount = 0;
//        double lastLimelightPollTime = 0;
//        com.qualcomm.hardware.limelightvision.LLResult cachedResult = null;
//        Pose lastLimelightPose = null; // To store the last successfully calculated pose from Limelight
//
//        boolean firstRun = true;
//
//        while (opModeIsActive()) {
//            // ═══════════════════════════════════════════════════
//            // DRIVING AND LOCALIZATION UPDATE
//            // ═══════════════════════════════════════════════════
//            robot.chassisLocal.update();
//            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            if(firstRun) {
//                Log.w("robot", "chassis localization has been updated");
//            }
//            // ═══════════════════════════════════════════════════
//            // LIMELIGHT MEGATAG1 RELOCALIZATION
//            // ═══════════════════════════════════════════════════
////
//            // One-time relocalization: poll only when a button is pressed
//            boolean wantRelocalize = (gamepad1.dpad_down && !lastDpadDown)
//                    || (gamepad1.touchpad && !lastTouchpad);
//            if (wantRelocalize) {
//                cachedResult = robot.vision.getLimelight().getLatestResult();
//                lastLimelightPollTime = getRuntime();
//            }
//
//            if(firstRun) {
//                Log.w("localiztion", "relocalized completed");
//            }
//
//            if (gamepad1.dpad_down && !lastDpadDown && (cachedResult != null)) {
//                // DPAD DOWN = Position only (one-time)
//                com.pedropathing.geometry.Pose poseCorrection = relocalization.getRelocalizationPose(
//                        robot.chassisLocal.getPose(),
//                        cachedResult,
//                        false, // includeHeading = false
//                        robot.turret.getRotatorPos());
//                if (poseCorrection != null) {
//                    robot.chassisLocal.setPose(poseCorrection);
//                    lastLimelightPose = poseCorrection;
//                    gamepad1.rumble(250);
//                }
//            }
//
//            if ((gamepad1.touchpad && !lastTouchpad) && (cachedResult != null)) {
//                // TOUCHPAD = Position + heading (one-time full snap)
//                com.pedropathing.geometry.Pose fullResnap = relocalization.getRelocalizationPose(
//                        robot.chassisLocal.getPose(),
//                        cachedResult,
//                        true, // includeHeading = true
//                        robot.turret.getRotatorPos());
//                if (fullResnap != null) {
//                    robot.chassisLocal.setPose(fullResnap);
//                    lastLimelightPose = fullResnap;
//                    gamepad1.rumble(500);
//                }
//            }
//
//            if (gamepad1.left_bumper && !lastLeftBumper) {
//                actions.resetEncoders();
//                gamepad1.rumble(500);
//            }
//
//            lastDpadDown = gamepad1.dpad_down;
//            lastTouchpad = gamepad1.touchpad;
//            lastLeftBumper = gamepad1.left_bumper;
//
//            // ═══════════════════════════════════════════════════
//            // MANUAL CONTROLS — SHOOTING (same as BlueTele)
//            // ═══════════════════════════════════════════════════
//
//            // Turret velocity
//            if (gamepad1.aWasPressed()) {
//                robot.turret.shiftVelocity(launcherSpeedIncrement);
//            }
//            if (gamepad1.yWasPressed()) {
//                robot.turret.shiftVelocity(-launcherSpeedIncrement);
//            }
//            if (gamepad1.rightStickButtonWasPressed()) {
//                robot.turret.presetVeloSwitch(veloSwitchNum);
//                veloSwitchNum += 1;
//                sillyControls = false;
//            }
//
//            // Hood control
//            actions.hoodControl(gamepad1.x, gamepad1.b);
//
//            // Rotator control
//            if (gamepad1.dpad_left) {
//                robot.turret.shiftRotator(-rotatorIncrement);
//                sillyControls = false;
//            }
//            if (gamepad1.dpad_right) {
//                robot.turret.shiftRotator(rotatorIncrement);
//                sillyControls = false;
//            }
//
//            // Feed (launch)
//            actions.launch(1, gamepad1.right_bumper, sillyTarget);
//            if (gamepad1.right_bumper) {
//                gamepad1.rumble(100);
//            }
//
//            // Intake — only when not launching (same as BlueTele)
//            double sumOfTrigs = gamepad1.left_trigger - gamepad1.right_trigger;
//            if (!gamepad1.right_bumper) {
//                actions.intake(sumOfTrigs);
//            }
//
//            // ═══════════════════════════════════════════════════
//            // AUTOMATIC CONTROLS (same as BlueTele)
//            // ═══════════════════════════════════════════════════
//
//            if (gamepad1.leftStickButtonWasPressed()) {
//                sillyControls = !sillyControls;
//            }
//
//            // Manual relocalization (DPAD UP — same as BlueTele)
//            if (gamepad1.dpad_up && !lastDpadUp) {
//                actions.HumanPLayerFix(dpadUpHeading);
//            }
//            //
//            lastDpadUp = gamepad1.dpad_up;
//
//            // Dynamic shooting
//            sillyTarget = robot.chassisLocal.sillyTargetPose(target);
//            inPosition = robot.chassisLocal.isShootingPosition();
//            if (sillyControls) {
//                if (inPosition) {
//                    actions.aimRotatorLocal(sillyTarget, telemetry);
//                    actions.adjustShootingParams(sillyTarget);
//                } else {
//                    robot.turret.setVelocity(1100);
//                }
//
//            }
//
//            // ═══════════════════════════════════════════════════
//            // TELEMETRY
//            // ═══════════════════════════════════════════════════
//
//            // Relocalization status (new)
//            telemetry.addLine("═══ RELOCALIZATION ═══");
//            telemetry.addData("Relocalize", "DPAD=pos TOUCHPAD=pos+heading");
//            relocalization.addTelemetry(telemetry, robot.chassisLocal.getPose());
//
//            // Limelight data (optimized to use the cached result)
//            telemetry.addLine("");
//            telemetry.addLine("LIMELIGHT DATA");
//            boolean bool = (cachedResult != null && cachedResult.isValid());
//            if (bool) {
//                telemetry.addLine("Passed boolean");
//                telemetry.update();
//                // To get true distance to the AprilTag, we use the camera-to-target 3D
//                // translation vector.
//                java.util.List<com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult> fiducials = cachedResult
//                        .getFiducialResults();
//                if (fiducials != null && !fiducials.isEmpty()) {
//                    com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult topTag = fiducials.get(0);
//                    org.firstinspires.ftc.robotcore.external.navigation.Pose3D camPose = topTag
//                            .getCameraPoseTargetSpace();
//
//                    telemetry.addData("Tag ID Seen", topTag.getFiducialId());
//
//                    if (camPose != null) {
//                        double xMeters = camPose.getPosition()
//                                .toUnit(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER).x;
//                        double yMeters = camPose.getPosition()
//                                .toUnit(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER).y;
//                        double zMeters = camPose.getPosition()
//                                .toUnit(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER).z;
//
//                        // 3D Distance (the hypotenuse in space, like a tape measure from lens to tag)
//                        double distance3DMeters = Math.sqrt(xMeters * xMeters + yMeters * yMeters + zMeters * zMeters);
//                        double distance3DInches = distance3DMeters * 39.3701;
//
//                        // 2D Ground Distance (ignoring height differences, matching the Pedro Pathing
//                        // flat plane)
//                        double dist2D = Math.sqrt(xMeters * xMeters + zMeters * zMeters) * 39.3701;
//
//                        telemetry.addData("Limelight 3D Dist (Tape Measure)", "%.1f in", distance3DInches);
//                        telemetry.addData("Limelight 2D Dist (Flat Floor)", "%.1f in", dist2D);
//                    } else {
//                        telemetry.addData("Limelight 3D Dist", "N/A (No Pose3D in Fiducial)");
//                    }
//                } else {
//                    telemetry.addData("Limelight 3D Dist", "N/A (No Fiducials)");
//                }
//            }
//            telemetry.addData("Limelight?", bool);
//
//            // Position data
//            telemetry.addLine("");
//            telemetry.addLine("ROBOT POSITION");
//            telemetry.addData("X", "%.1f", robot.chassisLocal.getPose().getX());
//            telemetry.addData("Y", "%.1f", robot.chassisLocal.getPose().getY());
//            telemetry.addData("Heading", "%.1f°", Math.toDegrees(robot.chassisLocal.getPose().getHeading()));
//
//            // Hardware data
//            telemetry.addLine("");
//            telemetry.addLine("HARDWARE STATUS");
//            telemetry.addData("Hood", "%.3f", robot.turret.getHoodPos());
//                    LimelightRelocalization.rotatorTicksToDegrees(robot.turret.getRotatorPos()));
//            telemetry.addData("Intake power", "%.2f", robot.intake.getPower());
//            telemetry.addData("Launcher velocity", robot.turret.getVelocity());
//            telemetry.addData("Distance", robot.chassisLocal.getDistance(target));
//            telemetry.update();
//            //
//
//            if(firstRun) {
//                Log.w("teleop", "teleop loop completed");
//            }
//
//            firstRun = false;
//        }
//
//        // ── CLEANUP (Crucial for preventing stop() hangs in LinearOpMode) ──
//        // The Limelight runs a background VisionPortal thread. If we don't explicitly
//        // stop it when the OpMode ends, the FTC SDK gets stuck waiting for that thread
//        // to die, causing the "stuck in stop()" error.
//
//        telemetry.addLine("Stopping Limelight...");
//        telemetry.update();
//        try {
//            if (robot.vision.getLimelight() != null) {
//                robot.vision.getLimelight().stop();
//            }
//        } catch (Exception e) {
//            // Ignore any shutdown errors
//        }
//    }
//}