package org.firstinspires.ftc.teamcode.PIDFTuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Drive Motor Configurator", group = "Tuners")
public class DriveMotorConfigurator extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.yWasPressed()){
                leftFront.setPower(1);
            }

            telemetry.update();
        }
    }
}


