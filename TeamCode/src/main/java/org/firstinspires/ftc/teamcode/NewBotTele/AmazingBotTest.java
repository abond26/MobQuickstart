package org.firstinspires.ftc.teamcode.NewBotTele;

import android.util.Log;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.TestTurret;
import org.firstinspires.ftc.teamcode.robotControl.RedUniversalConstants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(name = "AmazingBotTest")
public class AmazingBotTest extends LinearOpMode implements RedUniversalConstants {
    Robot robot;
    RobotActions actions;
    TestTurret testTurret;
    boolean sillyControls = false;

    Pose sillyTarget;



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

        while (opModeIsActive()) {

            robot.chassisLocal.update();
            sillyTarget = robot.chassisLocal.sillyTargetPose(target);

            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            double currentDist = robot.chassisLocal.getDistance(target);
            testTurret.update(currentDist);

            if (gamepad1.aWasPressed()) testTurret.shiftVelocity(50);
            if (gamepad1.yWasPressed()) testTurret.shiftVelocity(-50);
            

            if (gamepad1.dpad_left) {
                robot.turret.shiftHood(-0.005);
            }
            if (gamepad1.dpad_right){
                robot.turret.shiftHood(0.005);
            }
            if (gamepad1.leftStickButtonWasPressed()) {
                sillyControls = !sillyControls;
            }

            actions.launch(1, gamepad1.right_bumper);
            if (gamepad1.right_bumper) {
                gamepad1.rumble(100);
            }
            double sumOfTrigs = gamepad1.left_trigger - gamepad1.right_trigger;
            if (!gamepad1.right_bumper) {
                actions.intake(sumOfTrigs);
            }
            if (sillyControls) {
                actions.aimRotatorLocal(target, telemetry);
                actions.autoVelocityEquation(sillyTarget);
            }
            // Telemetry
            telemetry.addData("Mode", "MANUAL PID LOOP");
            telemetry.addData("RPM", "%.1f / Target: %.1f", testTurret.getRPM(), testTurret.getTargetVelocity());
            telemetry.addData("Hood", "%.3f", robot.turret.getHoodPos());
            telemetry.addData("Distance", "%.2f", currentDist);
            telemetry.update();
        }
    }
}
