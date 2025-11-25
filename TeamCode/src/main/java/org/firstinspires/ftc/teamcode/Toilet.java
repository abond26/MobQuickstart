package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


//@Config
@TeleOp
public class Toilet extends LinearOpMode {
    private Limelight3A limelight;
    double time;
    double newTime;
    boolean xLast = false;
    //x is 0.9
    boolean yLast = false;
    //y is 1
    boolean aLast =false;
    //a is 0.6

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotor intake, launcher, flicker, rotator;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

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
        rotator.setPower(0);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        follower.startTeleopDrive();
        runtime.reset();
        while (opModeIsActive()){
            time = runtime.time();

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
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);




            boolean xPressed = gamepad1.x && !xLast;
            xLast = gamepad1.x;

            boolean yPressed = gamepad1.y && !yLast;
            yLast = gamepad1.y;

            boolean aPressed = gamepad1.a && !aLast;
            aLast = gamepad1.a;

            if (aPressed){
                newTime = runtime.time();
                launcher.setPower(0.7);
                while (newTime-time<1.5){
                    newTime = runtime.time();
                }
                intake.setPower(-1);
                flicker.setPower(1);
                while (newTime-time<2){
                    newTime = runtime.time();
                }
                intake.setPower(0);
                flicker.setPower(0);
                while (newTime-time<3){
                    newTime = runtime.time();
                }
                intake.setPower(-1);
                flicker.setPower(1);
                while (newTime-time<4){
                    newTime = runtime.time();
                }
                intake.setPower(0);
                flicker.setPower(0);
                while (newTime-time<5){
                    newTime = runtime.time();
                }
                intake.setPower(-1);
                flicker.setPower(1);


            }



            if (xPressed){
                newTime = runtime.time();
                launcher.setPower(0.93);
                while (newTime-time<1.5){
                    newTime = runtime.time();
                }
                intake.setPower(-1);
                flicker.setPower(1);
                while (newTime-time<2){
                    newTime = runtime.time();
                }
                intake.setPower(0);
                flicker.setPower(0);
                while (newTime-time<3){
                    newTime = runtime.time();
                }
                intake.setPower(-1);
                flicker.setPower(1);
                while (newTime-time<4){
                    newTime = runtime.time();
                }
                intake.setPower(0);
                flicker.setPower(0);
                while (newTime-time<5){
                    newTime = runtime.time();
                }
                intake.setPower(-1);
                flicker.setPower(1);


            }

            if (yPressed){
                newTime = runtime.time();
                launcher.setPower(1);
                while (newTime-time<1.5){
                    newTime = runtime.time();
                }
                intake.setPower(-1);
                flicker.setPower(1);
                while (newTime-time<2){
                    newTime = runtime.time();
                }
                intake.setPower(0);
                flicker.setPower(0);
                while (newTime-time<2.5){
                    newTime = runtime.time();
                }
                intake.setPower(-1);
                flicker.setPower(1);
                while (newTime-time<3){
                    newTime = runtime.time();
                }
                intake.setPower(0);
                flicker.setPower(0);
                while (newTime-time<3.5){
                    newTime = runtime.time();
                }
                intake.setPower(-1);
                flicker.setPower(1);


            }



            //intake movements
            if (gamepad1.left_bumper){
                intake.setPower(1);
            }
            else if (gamepad1.right_bumper){
                intake.setPower(-1);
            }
            else {
                intake.setPower(0);
            }
            //launcher movements
            launcher.setPower(gamepad1.right_trigger);
            flicker.setPower(gamepad1.left_trigger);


            if (gamepad2.dpad_left){
                rotator.setPower(-0.2);
            }
            else if (gamepad2.dpad_right){
                rotator.setPower(0.2);
            }
            else {
                rotator.setPower(0);
            }

            telemetry.update();
        }
    }
    //for organization. Sets both intakes' power in the same direction

}