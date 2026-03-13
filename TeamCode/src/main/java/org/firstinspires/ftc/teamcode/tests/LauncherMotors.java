package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class LauncherMotors extends LinearOpMode {
    DcMotorEx launcherL, launcherR;

    @Override
    public void runOpMode() throws InterruptedException{
        launcherL = hardwareMap.get(DcMotorEx.class, "launcherL");
        launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherL.setVelocity(0);

        launcherR = hardwareMap.get(DcMotorEx.class, "launcherR");
        launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherR.setVelocity(0);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up){
                launcherL.setVelocity(launcherL.getVelocity()+1);
                launcherR.setVelocity(launcherL.getVelocity()+1);
            }
            if (gamepad1.dpad_down){
                launcherL.setVelocity(launcherL.getVelocity()-1);
                launcherR.setVelocity(launcherL.getVelocity()-1);
            }
            if (gamepad1.x){
                launcherL.setVelocity(0);
                launcherR.setVelocity(0);
            }

            telemetry.addData("LauncherL power", launcherL.getVelocity());
            telemetry.addData("LauncherR power", launcherR.getVelocity());
            telemetry.update();



        }
    }
}
