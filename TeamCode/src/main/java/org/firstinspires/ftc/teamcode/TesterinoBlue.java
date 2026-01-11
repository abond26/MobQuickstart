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
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@TeleOp

public class TesterinoBlue extends LinearOpMode {
    double newTime;
    double time;


    //282
    //-301
    double sumOfTrigs;
    boolean yLast = false;
    boolean aLast =false;
    boolean xLast = false;
    int motor180Range = 630;
    int limelightUpAngle = 20;
    private int limeHeight = 35;
    private int tagHeight = 75;
    private int y = tagHeight - limeHeight;
    public static double driveMultiplier = 0.7;
    private Limelight3A limelight;
    public ElapsedTime runtime = new ElapsedTime();
    private Servo hood;

    // Distance threshold for hood adjustment (tune this value)
    private static final double DISTANCE_THRESHOLD = 180.0; // Example: change hood when distance > 100 inches
    private static final double CLOSE_HOOD_POSITION = .79; // Hood position for close shots
    private static final double FAR_HOOD_POSITION = 0.4204; // Hood position for far shots
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotor intake, flicker, rotator, theWheelOfTheOx;
    private DcMotorEx jollyCrusader;
    private Follower follower;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public void runOpMode() throws InterruptedException{
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0,0.0328);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        jollyCrusader = hardwareMap.get(DcMotorEx.class, "launcher");
        jollyCrusader.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jollyCrusader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jollyCrusader.setVelocity(0);
        //jollyCrusader.setDirection(DcMotorSimple.Direction.REVERSE);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setDirection(DcMotorSimple.Direction.REVERSE);
        rotator.setPower(1);

        intake = hardwareMap.get(DcMotor.class, "tree");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        theWheelOfTheOx.setPower(0);
        theWheelOfTheOx.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
            telemetry.addData("LL", "initialized");
        } else {
            telemetry.addData("LL", "not found");
        }

        //dradle

        waitForStart();
        follower.startTeleopDrive();
        while (opModeIsActive()){
            time = runtime.time();
            boolean yPressed = gamepad1.y && !yLast;
            yLast = gamepad1.y;

            boolean aPressed = gamepad1.a && !aLast;
            aLast = gamepad1.a;
            boolean xPressed = gamepad2.x && !xLast;
            xLast = gamepad2.x;
//            if (gamepad1.x){
//                limelight.pipelineSwitch(1);
//            }
//            if (gamepad1.b){
//                limelight.pipelineSwitch(0);
//            }

            drive();



            //launcha
            if (aPressed){
                jollyCrusader.setVelocity(jollyCrusader.getVelocity()+50);
            }
            if (yPressed){
                jollyCrusader.setVelocity(jollyCrusader.getVelocity()-50);
            }




            //feed the flame ._.
            if (gamepad1.right_bumper){
                theWheelOfTheOx.setPower(1);


            }
            else if (gamepad1.left_bumper){
                theWheelOfTheOx.setPower(-1);
            }
            else {
                theWheelOfTheOx.setPower(0);
            }

            //intake
            sumOfTrigs = gamepad1.left_trigger-gamepad1.right_trigger;
            if (sumOfTrigs!=0){
                intake(sumOfTrigs);
            } else {
                intake.setPower(0);
            }



            //rotator
            if (gamepad1.dpad_left){
                rotator.setTargetPosition(rotator.getCurrentPosition()-30);
            }
            else if (gamepad1.dpad_right){
                rotator.setTargetPosition(rotator.getCurrentPosition()+30);
            }
            else{
                rotator.setTargetPosition(rotator.getCurrentPosition());
            }

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
                    if (!gamepad1.dpad_right && !gamepad1.dpad_left){
                        adjustRotator(txDeg);
                    }

                }

                double currentDistance = getDist(tyDeg);
                telemetry.addData("Distance", currentDistance);

                if (gamepad1.right_stick_button && currentDistance > 0){
                    jollyCrusader.setVelocity(calcVelocity(currentDistance));
                    adjustHoodBasedOnDistance(currentDistance);
                }


            }
            if (gamepad1.left_stick_button){
                jollyCrusader.setVelocity(0);
            }


            if (gamepad1.x){
                hood.setPosition(hood.getPosition()-0.005);
            }
            if (gamepad1.b){
                hood.setPosition(hood.getPosition()+0.005);
            }


            telemetry.addData("jolly crusader velocity", jollyCrusader.getVelocity());
            telemetry.addData("rotator pos", rotator.getTargetPosition());
            telemetry.addData("hood pos", hood.getPosition());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.update();


        }


    }

    public void drive(){
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
        leftRear.setPower(leftRearPower*driveMultiplier);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower*driveMultiplier);
    }
    public void adjustRotator(double tx) {
        double fracOfFullCircum = Math.toRadians(tx) / (2 * Math.PI);
        int adjustment = (int) (fracOfFullCircum * motor180Range * 2);
        int newPosition = rotator.getCurrentPosition() + adjustment - 28;
        rotator.setTargetPosition(newPosition);
    }

    public double getDist(double tyDeg) {
        double tyRad = Math.abs(Math.toRadians(tyDeg+limelightUpAngle));
        double dist = y / Math.tan(tyRad);
        double realDist = 0.55*dist+40.3;
        telemetry.addData("angle", Math.toDegrees(tyRad));
        telemetry.addData("fakeDist", dist);
        telemetry.addData("realDist", realDist);
        return realDist;
    }
    public double calcVelocity(double dist) {
        double velocity = 3.30933 * dist + 1507.01002;
        return velocity;
    }
    public void intake(double intakePower){
        intake.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-0.3);
        }
    }



    public void adjustHoodBasedOnDistance(double distance) {
        if (hood != null) {
            if (distance > DISTANCE_THRESHOLD) {
                hood.setPosition(FAR_HOOD_POSITION);
            } else {
                hood.setPosition(CLOSE_HOOD_POSITION);
            }
        }
    }

}