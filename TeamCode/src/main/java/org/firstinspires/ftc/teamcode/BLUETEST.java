package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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


//@Config
//@TeleOp
public class  BLUETEST extends LinearOpMode {
    //time
    private static double reaccelerationTime = 0.4; //determines how long between shooting
    public static double speedUpWait = 1.5; //how long we wait before the motor is up to speed
    public static double shootTime1 = 1.6; //how long we push the balls through the turret for
    public static double reaccelerateWait1 = shootTime1+reaccelerationTime; //how long we wait before the motor is up to speed again
    public static double shootTime2 = 2.1; //how long we push the balls through the turret for
    public static double reaccelerateWait2 = shootTime2+reaccelerationTime; //how long we wait before the motor is up to speed again
    public static double shootTime3 = 2.6;
    public static double driveMultiplier = 0.7;


    private int limeHeight = 33;
    private int tagHeight = 75;
    private int y = tagHeight - limeHeight;
    //Rotator var
    int motor180Range = 910;
    int limelightUpAngle = 15;
    private int vMultiplier = 9;
    private Limelight3A limelight;
    double time;
    double newTime;
    boolean xLast = false;
    //x is 0.9
    boolean yLast = false;
    //y is 1
    boolean aLast =false;
    //a is 0.6
    private int rotatorSpeed = 50;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotor intake, flicker, rotator;
    private DcMotorEx launcher;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "tree");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        flicker = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        flicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flicker.setPower(0);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }

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
            leftRear.setPower(leftRearPower*driveMultiplier);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower*driveMultiplier);

            boolean xPressed = gamepad1.x && !xLast;
            xLast = gamepad1.x;

            boolean yPressed = gamepad1.y && !yLast;
            yLast = gamepad1.y;

            boolean aPressed = gamepad1.a && !aLast;
            aLast = gamepad1.a;


            //Limelight calibration
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
                if (yPressed) {
                    launcher.setVelocity(launcher.getVelocity()+30);
                }
                if (aPressed) {
                    launcher.setVelocity(launcher.getVelocity()-30);
                }
                if (gamepad1.b) {
                    launcher.setPower(calcVelocity(getDist(tyDeg)));
                }

                telemetry.addData("dist", getDist(tyDeg));
                telemetry.addData("CalcVelocity", calcVelocity(getDist(tyDeg)));

                if (gamepad1.dpad_left){
                    rotator.setTargetPosition(rotator.getCurrentPosition()-rotatorSpeed);
                }
                if (gamepad1.dpad_right) {
                    rotator.setTargetPosition(rotator.getCurrentPosition() + rotatorSpeed);
                }
                if (gamepad1.right_trigger > 0.1){
                    launcher.setPower(0);
                }


                if (xPressed){
                    newTime = runtime.time();
                    launcher.setVelocity(calcVelocity(getDist(tyDeg)));
                    while (newTime-time<speedUpWait){
                        launcher.setVelocity(calcVelocity(getDist(tyDeg)));
                        drive();
                        newTime = runtime.time();
                    }
                    intake.setPower(-1);
                    flicker.setPower(1);
                    while (newTime-time<shootTime1){
                        launcher.setVelocity(calcVelocity(getDist(tyDeg)));
                        drive();
                        newTime = runtime.time();
                    }
                    intake.setPower(0);
                    flicker.setPower(0);
                    while (newTime-time<reaccelerateWait1){
                        launcher.setVelocity(calcVelocity(getDist(tyDeg)));
                        drive();
                        newTime = runtime.time();
                    }
                    intake.setPower(-1);
                    flicker.setPower(1);
                    while (newTime-time<shootTime2){
                        launcher.setVelocity(calcVelocity(getDist(tyDeg)));
                        drive();
                        newTime = runtime.time();
                    }
                    intake.setPower(0);
                    flicker.setPower(0);
                    while (newTime-time<reaccelerateWait2){
                        launcher.setVelocity(calcVelocity(getDist(tyDeg)));
                        drive();
                        newTime = runtime.time();
                    }
                    intake.setPower(-1);
                    flicker.setPower(1);
                    while (newTime-time<shootTime3){
                        drive();
                        launcher.setVelocity(calcVelocity(getDist(tyDeg)));
                        newTime = runtime.time();
                    }


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

                flicker.setPower(gamepad1.left_trigger);
                telemetry.addData("Launcher velocity (ticks/sec)", launcher.getVelocity());
            }



            // Removed - was conflicting with RUN_TO_POSITION mode by setting power to 0
            // Use gamepad1 dpad_left/right instead for rotator control
            telemetry.addData("motorPos", rotator.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("motorPos", rotator.getCurrentPosition());
        telemetry.update();
    }
    public void adjustRotator(double tx) {
        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        int newPosition = rotator.getCurrentPosition() + adjustment + 5;
        rotator.setTargetPosition(newPosition);
    }

    public double getDist(double tyDeg) {
        double tyRad = Math.toRadians(tyDeg+limelightUpAngle);
        double dist = y / Math.tan(tyRad);
        return dist;
    }

    public double calcVelocity(double dist) {
        double rice = dist/654.83484;
        double velocity = 1149.3757*Math.pow(2.72,rice)+ 83.439116;
        double rpower = velocity/2580;
        return rpower;

    }

    public void drive(){
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
    }


}
