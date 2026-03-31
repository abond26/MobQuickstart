package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "RotatorServoTest", group = "test")
public class RotatorServoTest extends LinearOpMode {
    private Servo rotator;
    private double position = 0.0;
    private static final double STEP_FINE = 0.001;
    private static final double STEP_COARSE = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        rotator = hardwareMap.get(Servo.class, "rotator");
        rotator.setDirection(Servo.Direction.REVERSE);

        rotator.setPosition(0.0);
        position = 0.0;

        telemetry.addData("Status", "Initialized — Rotator zeroed to 0.0");
        telemetry.addData("Controls", "DPad L/R = fine, Bumpers = coarse, A = zero");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Fine adjustment with dpad
            if (gamepad1.dpad_right) {
                position += STEP_FINE;
            }
            if (gamepad1.dpad_left) {
                position -= STEP_FINE;
            }

            // Coarse adjustment with bumpers
            if (gamepad1.right_bumper) {
                position += STEP_COARSE;
            }
            if (gamepad1.left_bumper) {
                position -= STEP_COARSE;
            }

            // Reset to zero
            if (gamepad1.a) {
                position = 0.0;
            }
            // Reset to center (0.5)
            if (gamepad1.b) {
                position = 0.5;
            }

            // Clamp 0-1
            position = Math.max(0.0, Math.min(1.0, position));

            rotator.setPosition(position);

            telemetry.addData("Rotator Position", "%.4f", position);
            telemetry.addData("", "------- Controls -------");
            telemetry.addData("DPad L/R", "Fine adjust (±%.3f)", STEP_FINE);
            telemetry.addData("Bumpers L/R", "Coarse adjust (±%.2f)", STEP_COARSE);
            telemetry.addData("A", "Zero (0.0)");
            telemetry.addData("B", "Center (0.5)");
            telemetry.update();
        }
    }
}
