package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoResetClass extends LinearOpMode {
    private Servo intakeShifterR, intakeShifterL,blocker,hood;




    @Override
    public void runOpMode() throws InterruptedException{
        intakeShifterR = hardwareMap.get(Servo.class, "intakeShifterR");
        intakeShifterR.setDirection(Servo.Direction.REVERSE);
        intakeShifterL = hardwareMap.get(Servo.class, "intakeShifterL");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        waitForStart();
        while (opModeIsActive()) {
            intakeShifterR.setPosition(0);
            intakeShifterL.setPosition(0);
            hood.setPosition(0);
            blocker.setPosition(0);
            telemetry.addData("Hood Postion",hood.getPosition());
            telemetry.addData("intakeShifterR Postion", intakeShifterR.getPosition());
            telemetry.addData("intakeShifterL Postion", intakeShifterL.getPosition());
            telemetry.addData("Blocker Postion",blocker.getPosition());
            telemetry.update();


        }
    }
}