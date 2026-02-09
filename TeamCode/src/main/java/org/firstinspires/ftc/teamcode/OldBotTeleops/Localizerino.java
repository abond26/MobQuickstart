package org.firstinspires.ftc.teamcode.OldBotTeleops;

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
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp
public class Localizerino extends LinearOpMode {
    private Limelight3A limelight;
    int motor180Range = 590;
    private static final int DEGREES_270_TICKS = 630;
    int limelightUpAngle = 20;
    private int limeHeight = 35;
    private int tagHeight = 75;
    private int y = tagHeight - limeHeight;
    private Follower follower;
    private DcMotor rotator;
    private Pose startPose;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;
    Pose bluePos = new Pose(11, 137, 0);
    Pose redPos = new Pose(133, 137, 0);
    Pose target = bluePos;

    /** Previous turret angle (degrees) for unwrapping so we don't snap at ±180°. */
    private double lastTurretAngleDeg = 0;



    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        startPose = PoseStorage.loadPose(new Pose(24.4, 126.7, Math.toRadians(143)));
        follower.setStartingPose(startPose);

        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.update();

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

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setPower(1);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        follower.startTeleopDrive();













        while (opModeIsActive()) {
            follower.update();

            Pose robotPose = follower.getPose();
            double x = robotPose.getX();
            double y = robotPose.getY();
            double headingDeg = Math.toDegrees(robotPose.getHeading());

            double turretAngleDeg = alignTurret(x, y, headingDeg, target);

            drive();
            setRotatorToTurretAngle(turretAngleDeg);
            telemetry.addData("turret angle", turretAngleDeg);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", headingDeg);
            telemetry.update();
        }
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
        if (diff > 90) turretAngle -= 360;
        else if (diff < -90) turretAngle += 360;
        // Reset at 360° so we don't accumulate to 720°; command 0 and keep angle in [-180,180]
        if (turretAngle >= 180 || turretAngle <= -180) {
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


    public void adjustRotatorWithLocalization(double localizationAngleDeg) {
        // Check if rotator has reached 270 degrees, return to zero position
        int currentPos = rotator.getCurrentPosition();
        if (Math.abs(currentPos) >= DEGREES_270_TICKS) {
            rotator.setTargetPosition(0);
            return;
        }

        if (Math.abs(localizationAngleDeg) < 2.0) {
            rotator.setTargetPosition(currentPos);
            return;
        }


        double fracOfSemiCircum = Math.toRadians(localizationAngleDeg) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);

        // If direction is wrong, try negating the adjustment
        // The sign might be inverted depending on your coordinate system
        adjustment = adjustment; // Negate to fix direction (remove this line if it makes it worse)


        if (Math.abs(currentPos + adjustment) >= DEGREES_270_TICKS) {
            rotator.setTargetPosition(0);
            return;
        }

        int offset = 14;

        int newPosition = currentPos + adjustment - offset;
        rotator.setTargetPosition(newPosition);

        telemetry.addData("Using", "Localization Only");
        telemetry.addData("Loc Angle Deg", localizationAngleDeg);
        telemetry.addData("Frac of SemiCircum", fracOfSemiCircum);
        telemetry.addData("Adjustment Ticks", adjustment);
        telemetry.addData("Current Pos", currentPos);
        telemetry.addData("New Pos", newPosition);
    }
    public void adjustRotator(double turretAngle) {
        double fracOfSemiCircum = Math.toRadians(turretAngle) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        int newPosition = rotator.getCurrentPosition() + adjustment ;
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
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }
}