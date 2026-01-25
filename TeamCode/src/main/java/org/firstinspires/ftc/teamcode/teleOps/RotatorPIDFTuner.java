package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class RotatorPIDFTuner extends OpMode {
    public DcMotorEx rotator;
    public int leftPos = -400;
    public int rightPos = 400;
    int curPos = rightPos;
    double F = 0;
    double D = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init(){
        rotator = hardwareMap.get(DcMotorEx.class, "rotator");
        rotator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, D, F);
        rotator.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rotator.setPower(1);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop(){
        // get all gamepad commands
        // set target velocity
        // update telemetry

        if (gamepad1.yWasPressed()) {
            if (curPos == rightPos){
                curPos = leftPos;
            } else { curPos = rightPos; }
        }
        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.rightBumperWasPressed()){
            D += stepSizes[stepIndex];
        }
        if (gamepad1.leftBumperWasPressed()){
            D -= stepSizes[stepIndex];
        }

        //update PIDF
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        rotator.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set velocity
        rotator.setTargetPosition(curPos);

        double curRealPosition = rotator.getCurrentPosition();
        double error = curPos - curRealPosition;

        telemetry.addData("Target Position", curPos);
        telemetry.addData("Current Position", "%.2f", curRealPosition);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("--------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Tuning D", "%.4f (Bumpers L/R)", D);
        telemetry.addData("Step Size", "%.4f (B button)", stepSizes[stepIndex]);
    }
}
