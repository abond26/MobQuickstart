package org.firstinspires.ftc.teamcode.NewBotTele;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
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
import com.bylazar.telemetry.PanelsTelemetry;


@TeleOp(name = "BLUEEEEE b")
@Configurable

public class AmazingBotBlue extends LinearOpMode implements BlueUniversalConstants {
    Robot robot;
    RobotActions actions;
    TestTurret testTurret;
    boolean sillyControls = false;
    static TelemetryManager telemetryM;


    Pose sillyTarget;
    public static double RpmChange = 20;
    public static double intakespeed = 1;






    @Override
    public void runOpMode() throws InterruptedException {
        Pose startPose = PoseStorage.loadPose(defaultPose);
        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


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

        while (opModeIsActive()) {

//            if (gamepad1.right_stick_button){
//                PoseStorage.clearPose();
//            }

            robot.chassisLocal.update();
            sillyTarget = robot.chassisLocal.sillyTargetPose(target);

            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

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

            actions.launch(intakespeed, gamepad1.right_bumper);
            if (gamepad1.right_bumper) {
                gamepad1.rumble(100);
            }
            double sumOfTrigs = gamepad1.left_trigger - gamepad1.right_trigger;
            if (!gamepad1.right_bumper) {
                actions.intake(sumOfTrigs);
            }
            actions.ChangeTargBlue();
            Pose dynamicTarget = actions.getShootingTarget();
            sillyTarget = robot.chassisLocal.sillyTargetPose(dynamicTarget);
            if (sillyControls) {
                actions.aimRotatorLocal(dynamicTarget, telemetry);
                actions.autoVelocityEquation(dynamicTarget);
            }
//            Intake.DetectedColor detectedColor = robot.intake.getDetectedColor(telemetry);

//            PoseStorage.savePose(robot.chassisLocal.getPose());
            // Telemetry
            telemetryM.debug("Mode", "MANUAL PID LOOP");
            
            telemetryM.debug("RPM", "%.1f / Target: %.1f", testTurret.getRPM(), testTurret.getTargetVelocity());
            telemetryM.debug("Hood", "%.3f", robot.turret.getHoodPos());
//            telemetryM.debug("Ball Detected", detectedColor);
            telemetryM.debug("Target Pose", dynamicTarget);

            telemetryM.debug("Distance", "%.2f", currentDist);
            telemetryM.update();
        }
    }
}
