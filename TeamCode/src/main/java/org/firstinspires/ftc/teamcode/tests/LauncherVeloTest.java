package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LauncherVeloTest extends LinearOpMode {
    DcMotorEx launcherL, launcherR;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private DcMotor intake;
    private Servo hood;
    double shooterP = 164;
    double shooterF = 19.8;


    @Override
    public void runOpMode() throws InterruptedException{
        launcherL = hardwareMap.get(DcMotorEx.class, "launcherL");
        launcherL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(shooterP, 0, 0, shooterF);
        launcherL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        launcherR = hardwareMap.get(DcMotorEx.class, "launcherR");
        launcherR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(shooterP, 0, 0, shooterF);
        launcherR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);

        intake = hardwareMap.get(DcMotor.class, "tree");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);


        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        hood = hardwareMap.get(Servo.class, "hood");


        double targetPower = 0;
        boolean dpadUpWasPressed = false;
        boolean dpadDownWasPressed = false;
        hood.setPosition(0);


        waitForStart();

        while (opModeIsActive()) {
            double hoodPos = hood.getPosition();
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.dpad_up && !dpadUpWasPressed){
                targetPower += 100;
            }
            dpadUpWasPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !dpadDownWasPressed){
                targetPower -= 100;
            }
            dpadDownWasPressed = gamepad1.dpad_down;

            if (gamepad1.x){
                targetPower = 1400;
            } else if (gamepad1.b) {
                targetPower = 0.0;
            }
            if (gamepad2.x){
                double newPos = hoodPos + 0.0005;
                hood.setPosition(newPos);
            }
            if (gamepad2.b) {
                double newPos = hoodPos - 0.0005;
                hood.setPosition(newPos);
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


            telemetry.addData("LauncherL power", launcherL.getVelocity());
            telemetry.addData("LauncherR power", launcherR.getVelocity());
            telemetry.addData("Intake power", intake.getPower());
            telemetry.addData("Hood Postion" , hood.getPosition());
            telemetry.update();

        }

    }
    public void drive(double y, double x, double r) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);

        leftFront.setPower((y + x + r) / denominator);
        leftRear.setPower((y - x + r) / denominator );
        rightFront.setPower((y - x - r) / denominator);
        rightRear.setPower((y + x - r) / denominator);
    }}
