package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * PIDF Tuning OpMode for Rotator Motor
 * 
 * This OpMode helps you tune PIDF coefficients for your rotator motor
 * in RUN_TO_POSITION mode. Use this to optimize tracking performance.
 * 
 * Controls:
 * - D-Pad Up/Down: Adjust P (Proportional) gain
 * - D-Pad Left/Right: Adjust D (Derivative) gain  
 * - B Button: Change step size
 * - Y Button: Move rotator to test position (+200 ticks)
 * - A Button: Move rotator to test position (-200 ticks)
 * - X Button: Return rotator to center (0)
 * 
 * Tuning Guide:
 * 1. Start with P=10, I=0, D=0, F=0
 * 2. If rotator is slow to reach target: Increase P
 * 3. If rotator oscillates/overshoots: Decrease P, add D (start with D=1-5)
 * 4. If rotator settles slowly: Add small I (0.01-0.1), but be careful - I can cause instability
 * 5. F is usually 0 for position control
 */
@TeleOp(name = "Rotator PIDF Tuner", group = "Tuning")
public class RotatorPIDFTuner extends OpMode {
    
    private DcMotorEx rotator;
    
    // PIDF coefficients
    private double P = 10.0;  // Start with 10
    private double I = 0.0;   // Usually 0 for position
    private double D = 0.0;   // Add if overshooting
    private double F = 0.0;   // Usually 0 for position
    
    // Step sizes for tuning
    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    private int stepIndex = 1;  // Start with 1.0
    
    // Test positions
    private static final int TEST_POSITION_POSITIVE = 200;
    private static final int TEST_POSITION_NEGATIVE = -200;
    private static final int CENTER_POSITION = 0;
    
    // Button state tracking
    private boolean yLast = false;
    private boolean aLast = false;
    private boolean xLast = false;
    private boolean bLast = false;
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;
    private boolean dpadLeftLast = false;
    private boolean dpadRightLast = false;
    
    @Override
    public void init() {
        // Get rotator motor (must be DcMotorEx for PIDF)
        try {
            rotator = hardwareMap.get(DcMotorEx.class, "rotator");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Rotator must be DcMotorEx for PIDF tuning!");
            telemetry.addLine("Check hardware configuration.");
            telemetry.update();
            return;
        }
        
        // Initialize rotator
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator.setPower(1.0);
        
        // Set initial PIDF coefficients
        updatePIDF();
        
        telemetry.addLine("Rotator PIDF Tuner Initialized");
        telemetry.addLine("Use D-Pad to adjust P and D");
        telemetry.addLine("Use B to change step size");
        telemetry.addLine("Use Y/A/X to test positions");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Detect button presses
        boolean yPressed = gamepad1.y && !yLast;
        boolean aPressed = gamepad1.a && !aLast;
        boolean xPressed = gamepad1.x && !xLast;
        boolean bPressed = gamepad1.b && !bLast;
        boolean dpadUpPressed = gamepad1.dpad_up && !dpadUpLast;
        boolean dpadDownPressed = gamepad1.dpad_down && !dpadDownLast;
        boolean dpadLeftPressed = gamepad1.dpad_left && !dpadLeftLast;
        boolean dpadRightPressed = gamepad1.dpad_right && !dpadRightLast;
        
        // Update last states
        yLast = gamepad1.y;
        aLast = gamepad1.a;
        xLast = gamepad1.x;
        bLast = gamepad1.b;
        dpadUpLast = gamepad1.dpad_up;
        dpadDownLast = gamepad1.dpad_down;
        dpadLeftLast = gamepad1.dpad_left;
        dpadRightLast = gamepad1.dpad_right;
        
        // Change step size
        if (bPressed) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        
        // Adjust P (Proportional)
        if (dpadUpPressed) {
            P += stepSizes[stepIndex];
            updatePIDF();
        }
        if (dpadDownPressed) {
            P -= stepSizes[stepIndex];
            updatePIDF();
        }
        
        // Adjust D (Derivative)
        if (dpadRightPressed) {
            D += stepSizes[stepIndex];
            updatePIDF();
        }
        if (dpadLeftPressed) {
            D -= stepSizes[stepIndex];
            updatePIDF();
        }
        
        // Test positions
        if (yPressed) {
            rotator.setTargetPosition(TEST_POSITION_POSITIVE);
            telemetry.addLine("Moving to +200");
        }
        if (aPressed) {
            rotator.setTargetPosition(TEST_POSITION_NEGATIVE);
            telemetry.addLine("Moving to -200");
        }
        if (xPressed) {
            rotator.setTargetPosition(CENTER_POSITION);
            telemetry.addLine("Moving to center");
        }
        
        // Display telemetry
        displayTelemetry();
        telemetry.update();
    }
    
    /**
     * Update PIDF coefficients on the motor
     */
    private void updatePIDF() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        rotator.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
    }
    
    /**
     * Display tuning telemetry
     */
    private void displayTelemetry() {
        telemetry.addLine("=== Rotator PIDF Tuner ===");
        telemetry.addLine("--- Current PIDF Values ---");
        telemetry.addData("P (Proportional)", "%.4f (D-Pad U/D)", P);
        telemetry.addData("I (Integral)", "%.4f", I);
        telemetry.addData("D (Derivative)", "%.4f (D-Pad L/R)", D);
        telemetry.addData("F (Feedforward)", "%.4f", F);
        telemetry.addLine("--- Tuning Controls ---");
        telemetry.addData("Step Size", "%.4f (B button)", stepSizes[stepIndex]);
        telemetry.addLine("Y = +200, A = -200, X = Center");
        telemetry.addLine("--- Rotator Status ---");
        telemetry.addData("Current Position", rotator.getCurrentPosition());
        telemetry.addData("Target Position", rotator.getTargetPosition());
        telemetry.addData("Position Error", rotator.getTargetPosition() - rotator.getCurrentPosition());
        
        // Calculate velocity (approximate)
        int error = rotator.getTargetPosition() - rotator.getCurrentPosition();
        if (Math.abs(error) < 5) {
            telemetry.addLine("Status: SETTLED");
        } else if (Math.abs(error) < 20) {
            telemetry.addLine("Status: APPROACHING");
        } else {
            telemetry.addLine("Status: MOVING");
        }
        
        telemetry.addLine("--- Tuning Tips ---");
        telemetry.addLine("Slow? Increase P");
        telemetry.addLine("Oscillating? Decrease P, add D");
        telemetry.addLine("Settling slow? Try small I (0.01-0.1)");
    }
}
