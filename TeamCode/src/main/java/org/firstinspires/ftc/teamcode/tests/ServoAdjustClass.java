package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoAdjustClass extends LinearOpMode {
    private Servo intakeShifterR, intakeShifterL,blocker,hood;




    @Override
    public void runOpMode() throws InterruptedException{
        intakeShifterR = hardwareMap.get(Servo.class, "intakeShifterR");
        intakeShifterR.setDirection(Servo.Direction.FORWARD);
        intakeShifterL = hardwareMap.get(Servo.class, "intakeShifterL");
        intakeShifterL.setDirection(Servo.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.setDirection(Servo.Direction.REVERSE);
        intakeShifterR.setPosition(0);
        intakeShifterL.setPosition(0);
        hood.setPosition(0);
        blocker.setPosition(0);




        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                hood.setPosition(hood.getPosition()-0.0005);
            }
            if (gamepad1.dpad_right){
                hood.setPosition(hood.getPosition()+0.0005);
            }
            if (gamepad1.dpad_up) {
                blocker.setPosition(blocker.getPosition()+0.0005);
            }
            if (gamepad1.dpad_down){
                blocker.setPosition(blocker.getPosition()-0.0005);
            }
            if (gamepad1.left_bumper) {
                intakeShifterR.setPosition(intakeShifterR.getPosition()-0.0005);
                intakeShifterL.setPosition(intakeShifterR.getPosition()-0.0005);

            }
            if (gamepad1.right_bumper){
                intakeShifterR.setPosition(intakeShifterR.getPosition()+0.0005);
                intakeShifterL.setPosition(intakeShifterR.getPosition()+0.0005);
            }




            telemetry.addData("Hood Postion",hood.getPosition());
            telemetry.addData("intakeShifterR Postion", intakeShifterR.getPosition());
            telemetry.addData("intakeShifterL Postion", intakeShifterL.getPosition());
            telemetry.addData("Blocker Postion",blocker.getPosition());
            telemetry.update();


        }
    }
}