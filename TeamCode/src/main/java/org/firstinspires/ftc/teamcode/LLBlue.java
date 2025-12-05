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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class LLBlue extends LinearOpMode {

    //Height vars.
    private int rotatorSpeed = 20;
    private int limeHeight = 33;
    private int tagHeight = 75;
    private int y = tagHeight - limeHeight;
    //Rotator var
    int motor180Range = 910;
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    //Hardware
    private Limelight3A limelight;
    private DcMotor intake, launcher, flicker, rotator;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    boolean aLast = false;

    public void runOpMode() throws InterruptedException {

        // Create follower FIRST (it sets motor directions internally)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Get motor references AFTER follower creation so we use the same configured motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        launcher = hardwareMap.get(DcMotor.class, "launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPower(0);

        flicker = hardwareMap.get(DcMotor.class, "flicker");
        flicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flicker.setPower(0);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);

        waitForStart();
        follower.startTeleopDrive(); // Disabled - using manual drive instead

        while (opModeIsActive()) {

            boolean aPressed = gamepad1.x && !aLast;
            aLast = gamepad1.x;

            double y = -gamepad1.left_stick_y; //forward/backward
            double x = gamepad1.left_stick_x; //strafe (left/right)
            double r = gamepad1.right_stick_x; //rotate

            //bot movements
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
            double leftFrontPower = (y + x + r) / denominator;
            double leftRearPower = (y - x + r) / denominator;
            double rightFrontPower = (y - x - r) / denominator;
            double rightRearPower = (y + x - r) / denominator;

            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);



            if (limelight != null) {
                LLResult ll = limelight.getLatestResult();
                double txDeg = 0.0; //horizontal deg
                double tyDeg = 0.0; //vertical deg
                double ta = 0.0;
                boolean llValid = false;
                if (ll != null) {
                    txDeg = ll.getTx();
                    tyDeg = ll.getTy();
                    ta = ll.getTa();
                    llValid = ll.isValid();
                }


                if (llValid) {
                    telemetry.addData("Ta", ta);
                    telemetry.addData("tx", txDeg);
                    telemetry.addData("ty", tyDeg);
                    //adjustRotator(txDeg);

                }
                adjustRotator(txDeg);
                telemetry.addLine("Function: adjustRotator");
                telemetry.addData("dist", getDist(tyDeg, txDeg));
            }
            if (gamepad1.dpad_left){
                rotator.setTargetPosition(rotator.getCurrentPosition()-rotatorSpeed);
            }
            if (gamepad1.dpad_right){
                rotator.setTargetPosition(rotator.getCurrentPosition()+rotatorSpeed);
            }

            telemetry.addData("motorPos", rotator.getCurrentPosition());
            telemetry.update();
        }
    }

    public void adjustRotator(double tx) {
        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        int newPosition = rotator.getCurrentPosition() + adjustment;
        telemetry.addData("Adjustment", adjustment);
        telemetry.addData("New pos", newPosition);
        rotator.setTargetPosition(newPosition);
    }

    public double getDist(double tyDeg, double txDeg) {
        double tyRad = Math.toRadians(tyDeg);
        double dist = y / Math.tan(tyRad);
        return dist;
    }

}