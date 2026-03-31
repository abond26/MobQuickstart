package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class TjTest extends LinearOpMode {
    DcMotorEx launcherL, launcherR;
    private DcMotor intake;
    double shooterP = 400;
    double shooterF = 13.2965;


    @Override
    public void runOpMode() throws InterruptedException {
        launcherL = hardwareMap.get(DcMotorEx.class, "launcherL");
        launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, 0, 0, shooterF);
//        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        launcherR = hardwareMap.get(DcMotorEx.class, "launcherR");
        launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(shooterP, 0, 0, shooterF);
//        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);

        intake = hardwareMap.get(DcMotor.class, "tree");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);


        double targetPower = 0;
        boolean dpadUpWasPressed = false;
        boolean dpadDownWasPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !dpadUpWasPressed) {
                targetPower += 100;
            }
            dpadUpWasPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !dpadDownWasPressed) {
                targetPower -= 100;
            }
            dpadDownWasPressed = gamepad1.dpad_down;

            if (gamepad1.x) {
                targetPower = 1800;
            } else if (gamepad1.b) {
                targetPower = 0.0;
            }

            // Cap power at [-1.0, 1.0]
            //targetPower = Math.max(-1.0, Math.min(1.0, targetPower));

            launcherR.setVelocity(targetPower);
            launcherL.setVelocity(targetPower);

            // Intake control (triggers)
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-gamepad1.left_trigger);
            } else {
                intake.setPower(0);
            }

            // The Wheel Of The Ox control (buttons)



            telemetry.update();

        }
    }
}