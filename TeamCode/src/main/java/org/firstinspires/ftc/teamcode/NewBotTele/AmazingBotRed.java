package org.firstinspires.ftc.teamcode.NewBotTele;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;
import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TestTurret;
import org.firstinspires.ftc.teamcode.robotControl.RedUniversalConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(name = "Red Tele")
@Configurable

public class AmazingBotRed extends LinearOpMode implements RedUniversalConstants {
    Robot robot;
    RobotActions actions;
    TestTurret testTurret;
    boolean sillyControls = false;
    boolean inPosition = false;

    Pose sillyTarget;
    public static double RpmChange = 20;
    public static double intakespeed = 0.8;






    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = PoseStorage.loadPose(defaultPose);
        robot = new Robot(hardwareMap, startPose, PIPELINENUM);

        // Initialize the specialized TEST TURRET
        testTurret = new TestTurret(hardwareMap);

        actions = new RobotActions(
                robot.chassisLocal,
                robot.vision,
                testTurret,
                robot.gate,
                robot.intake);

        waitForStart();
        robot.chassisLocal.startTeleop();
        robot.turret.setRotatorPos(0.5);

        while (opModeIsActive()) {

//            if (gamepad1.right_stick_button){
//                PoseStorage.clearPose();
//            }

            robot.chassisLocal.update();
            sillyTarget = robot.chassisLocal.sillyTargetPose(target);

            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            double headingDeg = Math.toDegrees(robot.chassisLocal.getPose().getHeading());
            robot.vision.updateOrientation(headingDeg);

            if (gamepad1.touchpad) {
                Pose mt2Pose = robot.vision.getMegaTag2Pose();
                if (mt2Pose != null && robot.chassisLocal.getDistance(mt2Pose) < 15.0) {
                    robot.chassisLocal.setPose(mt2Pose);
                    gamepad1.rumble(250);
                }
            }

            double currentDist = robot.chassisLocal.getDistance(target);
            testTurret.update(currentDist);

            if (gamepad1.aWasPressed()) testTurret.shiftVelocity(RpmChange);
            if (gamepad1.yWasPressed()) testTurret.shiftVelocity(-RpmChange);


            if (gamepad1.dpad_left) {
                robot.turret.shiftHood(-0.005);
            }
            if (gamepad1.dpad_right){
                robot.turret.shiftHood(0.005);
            }
            if (gamepad1.leftStickButtonWasPressed()) {
                sillyControls = !sillyControls;
            }
            if (gamepad1.x) {
                robot.turret.shiftHood(-hoodIncrement);
            }
            if (gamepad1.b) {
                robot.turret.shiftHood(hoodIncrement);
            }
            if (gamepad1.dpad_up){
                robot.chassisLocal.setPose(ManualRelocal);
            }

            actions.launch(intakespeed, gamepad1.right_bumper);
            if (gamepad1.right_bumper) {
                gamepad1.rumble(100);
            }
            double sumOfTrigs = gamepad1.left_trigger - gamepad1.right_trigger;
            if (sumOfTrigs > 0.2){
                robot.intake.shift();
            }else{
                Intake.DetectedColor detectedColor = robot.intake.getDetectedColorByDistance(telemetry);
                telemetry.addData("Ball Detected", detectedColor);
            }
            if (!gamepad1.right_bumper) {
                actions.intake(sumOfTrigs);
            }

            inPosition = robot.chassisLocal.isShootingPosition();
            if (sillyControls && inPosition) {
                actions.ChangeTargRed();
                Pose dynamicTarget = actions.getShootingTarget();
//                actions.aimTurret(dynamicTarget);
                actions.aimRotatorLocal(dynamicTarget,telemetry);
            }

            else {
                testTurret.setRotatorPos(0.5);
            }

            Intake.DetectedColor detectedColor = robot.intake.getDetectedColorByDistance(telemetry);

//            PoseStorage.savePose(robot.chassisLocal.getPose());
            // Telemetry
            telemetry.addData("Mode", "MANUAL PID LOOP");
            telemetry.addData("RPM", "%.1f / Target: %.1f", testTurret.getRPM(), testTurret.getTargetVelocity());
            telemetry.addData("Hood", "%.3f", robot.turret.getHoodPos());
            telemetry.addData("Ball Detected", detectedColor);
            telemetry.addData("Pose", "X: %.1f, Y: %.1f", robot.chassisLocal.getPose().getX(), robot.chassisLocal.getPose().getY());
            telemetry.addData("Velo", "XVelo: %.1f, YVelo: %.1f", robot.chassisLocal.getVeloX(), robot.chassisLocal.getVeloY());

            telemetry.addData("Distance", "%.2f", currentDist);
            telemetry.update();
        }
    }
}
