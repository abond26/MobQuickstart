package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class idk extends LinearOpMode {

    // ===================== SHOOTER TIMING =====================
    private static double reaccelerationTime = 0.6;
    public static double speedUpWait = 1.5;
    public static double shootTime1 = 1.6;
    public static double reaccelerateWait1 = shootTime1 + reaccelerationTime;
    public static double shootTime2 = 2.2;
    public static double reaccelerateWait2 = shootTime2 + reaccelerationTime;
    public static double shootTime3 = 2.8;
    public static double driveMultiplier = 0.75;

    // ===================== LIMELIGHT CONFIG =====================
    private static final double TX_DEADBAND = 1.0;        // degrees
    private static final double ROTATOR_KP = 0.015;
    private static final double ROTATOR_MAX_POWER = 0.35;

    private double lastValidTx = 0;

    private int limeHeight = 33;
    private int tagHeight = 75;
    private int y = tagHeight - limeHeight;
    private int limelightUpAngle = 25;

    private Limelight3A limelight;

    // ===================== HARDWARE =====================
    private DcMotor intake, flicker, rotator;
    private DcMotorEx launcher;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        flicker = hardwareMap.get(DcMotor.class, "flicker");
        rotator = hardwareMap.get(DcMotor.class, "rotator");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
        }

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // ===================== DRIVE =====================
            double y = -gamepad2.left_stick_y; //forward/backward
            double x = gamepad2.left_stick_x; //strafe (left/right)
            double r = gamepad2.right_stick_x; //rotate

            //bot movements
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
            double leftFrontPower = (y + x + r) / denominator;
            double leftRearPower = (y - x + r) / denominator;
            double rightFrontPower = (y - x - r) / denominator;
            double rightRearPower = (y + x - r) / denominator;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower*driveMultiplier);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower*driveMultiplier);

            // ===================== LIMELIGHT AUTO AIM =====================
            boolean autoAim = gamepad1.right_bumper;

            if (limelight != null) {
                LLResult ll = limelight.getLatestResult();

                if (ll != null && ll.isValid()) {
                    double tx = ll.getTx();
                    double ty = ll.getTy();

                    lastValidTx = tx;

                    if (autoAim) {
                        adjustRotator(tx);
                    }

                    telemetry.addData("Dist", getDist(ty));
                    telemetry.addData("CalcVelocity", calcVelocity(getDist(ty)));
                } else if (autoAim) {
                    adjustRotator(lastValidTx);
                }
            }

            // ===================== MANUAL ROTATOR =====================
            if (!autoAim) {
                rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (gamepad1.dpad_left) {
                    rotator.setPower(-0.3);
                } else if (gamepad1.dpad_right) {
                    rotator.setPower(0.3);
                } else {
                    rotator.setPower(0);
                }
            }

            // ===================== INTAKE =====================

            if (gamepad1.left_bumper )
                intake.setPower(1);
            else if (gamepad1.right_bumper )
                intake.setPower(-1);
            else
                intake.setPower(0);


            // ===================== FLICKER =====================
            if (gamepad1.left_trigger > 0.1)
                flicker.setPower(gamepad1.left_trigger);
            else if (gamepad1.right_trigger > 0.1)
                flicker.setPower(-gamepad1.right_trigger);
            else
                flicker.setPower(0);

            telemetry.update();
        }
    }

    // ===================== AUTO AIM CONTROL =====================
    private void adjustRotator(double tx) {
        if (Math.abs(tx) < TX_DEADBAND) {
            rotator.setPower(0);
            return;
        }

        double power = Math.min(ROTATOR_MAX_POWER, Math.abs(tx) * ROTATOR_KP);
        int direction = tx > 0 ? 1 : -1;

        rotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotator.setPower(power * direction);
    }

    // ===================== DISTANCE =====================
    public double getDist(double tyDeg) {
        double tyRad = Math.toRadians(tyDeg + limelightUpAngle);
        return y / Math.tan(tyRad);
    }

    // ===================== SHOOTER VELOCITY =====================
    public double calcVelocity(double dist) {
        double rice = dist / 654.83484;
        double velocity = 1149.3757 * Math.pow(2.72, rice) + 83.439116;
        return velocity / 2580.0;
    }
}
