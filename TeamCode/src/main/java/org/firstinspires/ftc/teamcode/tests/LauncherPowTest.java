package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LauncherPowTest extends LinearOpMode {
    DcMotorEx launcherL, launcherR;
    private DcMotor intake,theWheelOfTheOx;


    @Override
    public void runOpMode() throws InterruptedException{
        launcherL = hardwareMap.get(DcMotorEx.class, "launcherL");
        launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherR = hardwareMap.get(DcMotorEx.class, "launcherR");
        launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake = hardwareMap.get(DcMotor.class, "tree");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);
        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theWheelOfTheOx.setPower(0);
        theWheelOfTheOx.setDirection(DcMotorSimple.Direction.REVERSE);


        double targetPower = 0;
        boolean dpadUpWasPressed = false;
        boolean dpadDownWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && !dpadUpWasPressed){
                targetPower += 0.05;
            }
            dpadUpWasPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !dpadDownWasPressed){
                targetPower -= 0.05;
            }
            dpadDownWasPressed = gamepad1.dpad_down;

            if (gamepad1.x){
                targetPower = 0.8;
            } else if (gamepad1.b) {
                targetPower = 0.0;
            }

            // Cap power at [-1.0, 1.0]
            targetPower = Math.max(-1.0, Math.min(1.0, targetPower));

            launcherR.setPower(targetPower);
//            launcherL.setPower(targetPower);

            // Intake control (triggers)
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
            }

            // The Wheel Of The Ox control (buttons)
            if (gamepad1.y) {
                theWheelOfTheOx.setPower(1.0); // Forward
            } else if (gamepad1.a) {
                theWheelOfTheOx.setPower(-1.0); // Reverse
            } else {
                theWheelOfTheOx.setPower(0); // Stop
            }

            telemetry.addData("LauncherL power", launcherL.getPower());
            telemetry.addData("LauncherR power", launcherR.getPower());
            telemetry.addData("Intake power", intake.getPower());
            telemetry.addData("WheelOfTheOx power", theWheelOfTheOx.getPower());
            telemetry.update();

        }
    }
}
