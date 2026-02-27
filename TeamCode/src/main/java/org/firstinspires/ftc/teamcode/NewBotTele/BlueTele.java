package org.firstinspires.ftc.teamcode.NewBotTele;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.robotControl.BlueUniversalConstants;

@TeleOp
public class BlueTele extends LinearOpMode implements BlueUniversalConstants {
    Pose sillyTarget;
    Robot robot;
    RobotActions actions;
    private int veloSwitchNum = 1;
    boolean autoControls = false;
    boolean sillyControls = false;

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

        waitForStart();
        robot.chassisLocal.startTeleop();


        int loopCount = 0;
        while (opModeIsActive()) {
            //Driving and updating position
            robot.chassisLocal.update();
            robot.chassisLocal.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


            //MANUAL CONTROLS
            //----------------------------------------------
            //Turret velocity
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


            //Hood control
            actions.hoodControl(gamepad1.x, gamepad1.b);

            //Rotator control
            if (gamepad1.dpad_left) {
                robot.turret.shiftRotator(-rotatorIncrement);
                autoControls = false;
            }
            if (gamepad1.dpad_right) {
                robot.turret.shiftRotator(rotatorIncrement);
                autoControls = false;
            }

            //Feed
            actions.launch(1, gamepad1.right_bumper);
            //Rumble on launch
            if (gamepad1.right_bumper){
                gamepad1.rumble(100);
            }

            //intake
            double sumOfTrigs = gamepad1.left_trigger-gamepad1.right_trigger;
            if (!gamepad1.right_bumper){
                actions.intake(sumOfTrigs);
            }



            //AUTOMATIC CONTROLS
            //----------------------------------------------

            //switch
//            if (gamepad1.touchpadWasPressed()){
//                autoControls = !autoControls;
//            }

            //Static shooting - technically we can remove this. Just here for testing
//            if (autoControls) {
//                actions.aimRotatorLocal(target, telemetry);
//                actions.adjustShootingParams(target);
//            }

            if (gamepad1.leftStickButtonWasPressed()) {
                sillyControls = !sillyControls;
            }

            if (gamepad1.touchpadWasPressed()) {
                actions.relocalizeBlue(aprilTagPose, telemetry);
            }

            //Dynamic shooting - also covers static shooting obv
            sillyTarget = robot.chassisLocal.sillyTargetPose(target);
            if (sillyControls & !autoControls ) {
                actions.aimRotatorLocal(sillyTarget, telemetry);
                actions.adjustShootingParams(sillyTarget);
            }

            telemetry.addLine("Automatic Telemetry");
            telemetry.addLine("--------------------------");
            telemetry.addData("Distance with Local", robot.chassisLocal.getDistance(target));
            telemetry.addData("X Position", robot.chassisLocal.getPose().getX());
            telemetry.addData("Y Position", robot.chassisLocal.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(robot.chassisLocal.getPose().getHeading()));



            telemetry.addLine("");
            telemetry.addLine("Basic Hardware Telemetry");
            telemetry.addLine("--------------------------");
            telemetry.addData("Hood position", robot.turret.getHoodPos());
            telemetry.addData("Rotator position", robot.turret.getRotatorPos());
            telemetry.addData("Intake power", robot.intake.getPower());
            telemetry.addData("Launcher velocity", robot.turret.getVelocity());
            telemetry.update();
        }
    }
}
