package org.firstinspires.ftc.teamcode.NewBotTele;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NewBotAuton.tangentialRed;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp

public class NewBotBlue extends LinearOpMode {
    double newTime;
    double time;
    double F = 12.35;
    double P = 132.5000;


    //282
    //-301
    double sumOfTrigs;
    boolean yLast = false;
    boolean aLast =false;
    boolean xLast = false;
    boolean rightBumperLast = false;
    ElapsedTime rightBumperTimer = new ElapsedTime();
    boolean rightBumperTimerStarted = false;
    private static final double HOOD_MOVE_DELAY_SECONDS = 0.5; // Time to hold button before hood moves
    int motor180Range = 1250;
    int limelightUpAngle = 20;
    private int limeHeight = 35;
    private int tagHeight = 75;
    private int y = tagHeight - limeHeight;
    public static double driveMultiplier = 1;
    private double lastTurretAngleDeg = 0;

    private Limelight3A limelight;
    public ElapsedTime runtime = new ElapsedTime();
    private Servo hood,blocker;

    // Distance threshold for hood adjustment (tune this value)
    private static final double FIRST_DISTANCE_THRESHOLD = 140.0;
    private static final double SECOND_DISTANCE_THRESHOLD = 200;
    private static final double CLOSE_HOOD_POSITION = 0.0339; // Hood position for close shots
    private static final double MID_HOOD_POSITION = 0.203+0.0167;
    private static final double FAR_HOOD_POSITION = 0.25+.0129; // Hood position for far shots
    private Pose startPose = new Pose(0, 0, 0);
    Pose bluePos = new Pose(11, 137, 0);
    Pose redPos = new Pose(133, 137, 0);
    Pose target = bluePos;
    private DcMotor intake, flicker, rotator, theWheelOfTheOx;
    private DcMotorEx jollyCrusader;
    private Follower follower;
    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public void runOpMode() throws InterruptedException{
        follower = Constants.createFollower(hardwareMap);
        startPose = PoseStorage.loadPose(new Pose(24.4, 126.7, Math.toRadians(143)));
        follower.setStartingPose(startPose);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.scaleRange(0,0.0761);
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0, 0.4);
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
        jollyCrusader.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        jollyCrusader.setDirection(DcMotorSimple.Direction.FORWARD);
        jollyCrusader.setVelocity(0);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        jollyCrusader.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //jollyCrusader.setDirection(DcMotorSimple.Direction.REVERSE);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            Pose robotPose = follower.getPose();
            double x = robotPose.getX();
            double y = robotPose.getY();
            double headingDeg = Math.toDegrees(robotPose.getHeading());
            double turretAngleDeg = alignTurret(x, y, headingDeg, target);
            time = runtime.time();
            boolean yPressed = gamepad1.dpad_down && !yLast;
            yLast = gamepad1.y;

            boolean aPressed = gamepad1.dpad_up && !aLast;
            aLast = gamepad1.a;
            boolean xPressed = gamepad2.x && !xLast;
            xLast = gamepad2.x;
            boolean rightBumperPressed = gamepad1.right_bumper && !rightBumperLast;
            rightBumperLast = gamepad1.right_bumper;

            if (rightBumperPressed) {
                rightBumperTimer.reset();
                rightBumperTimerStarted = true;
            }
//j
            if (!gamepad1.right_bumper) {
                rightBumperTimerStarted = false;
            }
            //            if (gamepad1.x){
//                limelight.pipelineSwitch(1);
//            }
//            if (gamepad1.b){
//                limelight.pipelineSwitch(0);
//            }

            drive();



            //launcha
            if (aPressed){
                jollyCrusader.setVelocity(jollyCrusader.getVelocity()+30);
            }
            if (yPressed){
                jollyCrusader.setVelocity(jollyCrusader.getVelocity()-30);
            }




            //feed the flame ._.
            if (gamepad1.right_bumper){
                blocker.setPosition(1);
                theWheelOfTheOx.setPower(1);
                intake.setPower(-1);
                gamepad1.rumble(100);


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
                blocker.setPosition(0);
                intake(sumOfTrigs);
            } else if (!gamepad1.right_bumper) {
                intake.setPower(0);
            }



            //rotator
            if (gamepad1.dpad_left){
                rotator.setTargetPosition(rotator.getCurrentPosition()-100);
            }
            else if (gamepad1.dpad_right){
                rotator.setTargetPosition(rotator.getCurrentPosition()+100);
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

                    if (llValid) {
                        telemetry.addLine("Limelight Detecting Yes");
                        telemetry.addData("Ta", ta);
                        telemetry.addData("tx", txDeg);
                        telemetry.addData("ty", tyDeg);
                        if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                            adjustRotator(txDeg, getDist(txDeg));
                        }
                    } else {
                        setRotatorToTurretAngle(turretAngleDeg);
                        telemetry.addLine("Limelight Detecting No");
                        telemetry.addLine("no data");
                    }
                }
                double currentDistance = getDist(tyDeg);
                telemetry.addData("Distance", currentDistance);

                if (gamepad1.right_stick_button && currentDistance > 0){
                    jollyCrusader.setVelocity(2300);
                }
//fasd

            }
            if (gamepad1.left_stick_button){
                jollyCrusader.setVelocity(1800);
            }
//cool


            if (gamepad1.x){
                hood.setPosition(hood.getPosition()-0.25);
            }
            if (gamepad1.b){
                hood.setPosition(hood.getPosition()+0.25);
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
    private double alignTurret(double x, double y, double headingDeg, Pose target) {
        double dx = target.getX() - x;
        double dy = target.getY() - y;
        double angleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngle = angleToGoal - headingDeg;
        while (turretAngle > 180) turretAngle -= 360;
        while (turretAngle < -180) turretAngle += 360;
        double normalized = turretAngle;
        double diff = turretAngle - lastTurretAngleDeg;
        if (diff > 180) turretAngle -= 360;
        else if (diff < -180) turretAngle += 360;
        // Reset at 360° so we don't accumulate to 720°; command 0 and keep angle in [-180,180]
        if (turretAngle >= 360 || turretAngle <= -360) {
            lastTurretAngleDeg = normalized;
            return 0;
        }
        lastTurretAngleDeg = turretAngle;
        return turretAngle;
    }
    private final int ROTATOR_ZERO_TICKS = 0;  // tick position when t8[888[urret faces forward; calibrate if needed

    public void setRotatorToTurretAngle(double turretAngleDeg) {
        double fracOf180 = Math.toRadians(turretAngleDeg) / Math.PI;
        int targetTicks = ROTATOR_ZERO_TICKS + (int) (fracOf180 * motor180Range);
        // Optional: clamp to physical limits (e.g. ±270°)
        rotator.setTargetPosition(targetTicks);
    }
    public void adjustRotator(double tx, double distance) {
        double fracOfFullCircum = Math.toRadians(tx) / (Math.PI);
        int adjustment = (int) (fracOfFullCircum * motor180Range);
        int offset = 0;
        if (distance > 200) {
            offset = 0;
        }
        int newPosition = rotator.getCurrentPosition() + adjustment - offset;
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
        double velocity;
        if (dist < FIRST_DISTANCE_THRESHOLD) {
            velocity = 4.94*dist + 1008;
        } else if (dist < SECOND_DISTANCE_THRESHOLD){
            velocity = 4.22*dist + 1129;
        }
        else{
            velocity = 16.66*dist - 1420;
        }
        return velocity;
    }
    public void intake(double intakePower){
        intake.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-1);
        }
    }


    public void adjustHoodBasedOnDistance(double dist) {
        if (dist < FIRST_DISTANCE_THRESHOLD) {
            hood.setPosition(CLOSE_HOOD_POSITION);
        } else if (dist < SECOND_DISTANCE_THRESHOLD){
            hood.setPosition(MID_HOOD_POSITION);
        }
        else{
            hood.setPosition(FAR_HOOD_POSITION);
        }
    }

}