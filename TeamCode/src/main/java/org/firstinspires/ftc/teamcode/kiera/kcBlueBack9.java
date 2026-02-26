package org.firstinspires.ftc.teamcode.kiera;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.Kishen.close9blue;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;





@Autonomous(name = "kc 9 blue back", group = "kc auton")
public class kcBlueBack9 extends OpMode {
    private int rotatorStartPosition=0;
    double txDeg = 0.0; //horizontal deg
    double tyDeg = 0.0; //vertical deg
    private Follower follower;

    // Flags to prevent path oscillation - ensure paths are only called once per state
    private boolean shoot2Started = false;
    private boolean shoot3Started = false;
    private boolean shoot4Started = false;
    private boolean shoot5Started = false;
    private boolean goAwayFromGateStarted = false;
    private boolean goTowardsGateStarted = false;
    private boolean opengateStarted = false;
    private boolean parkingStarted = false;

    private Servo hood;
    private int limeHeight = 33;
    private int offset = 28;
    private int tagHeight = 75;
    private static final double NORMAL_DRIVE_POWER = 1;
    private static final double INTAKE_DRIVE_POWER = 0.6; // tune this

    // Hood adjustment constants (from TesterinoBlue)
    private static final double DISTANCE_THRESHOLD = 180.0;
    private static final double CLOSE_HOOD_POSITION = .2541; // Hood position for close shots
    private static final double FAR_HOOD_POSITION = 0.36; // Hood position for far shots

    private int y = tagHeight - limeHeight;
    //Rotator var
    int motor180Range = 910;
    int limelightUpAngle = 25;
    private int vMultiplier = 9;
    private Limelight3A limelight;

    // Store last valid limelight values for fallback
    private double lastValidTx = 0.0;
    private double lastValidTy = 0.0;
    private double lastValidDistance = 0.0;
    private boolean hasValidLimelightData = false;

    private DcMotor leftFront, leftRear, rightFront, rightRear;


    private DcMotorEx launcher;
    private DcMotor tree, theWheelOfTheOx, rotator;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        START,
        SHOOT,
        INTAKE,
        SHOOT1,
        INTAKE1,
        INTAKE2,
        SHOOT2,
        RECORD,
        PARK


    }

    PathState pathState;

    private final Pose startPose= new Pose(60.734693877551024, 8.620408163265303,Math.toRadians(90));
    private final Pose shootPose = new Pose(60.53877551020408,15.12653061224489,Math.toRadians(110));

    private final Pose beforeIntakeOne= new Pose(60.5591836734694,32.66530612244898,Math.toRadians(180));
    private final Pose afterIntakeOne= new Pose(16.068954955554815,32.93200386190365,Math.toRadians(180));

   //go back to shooting position
    private final Pose beforeIntakeTwo =new Pose(60.599999999999994,55,Math.toRadians(180));
    private final Pose afterIntakeTwo =new Pose(16.387755102040824,55,Math.toRadians(180));
    //back to shooting position

    private final Pose recordIntake=new Pose(18.069387755102042,55.420408163265314,Math.toRadians(155));
//if it were in a good tuned robot it would be 55

    private PathChain firstShot,goToIntakeOne,intakeOne,backToShootOne,goToIntakeTwo,intakeTwo,backToShootTwo,goToIntakeRec;


    public void buildPaths(){
        firstShot=follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        goToIntakeOne=follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beforeIntakeOne))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beforeIntakeOne.getHeading())
                .build();

        intakeOne=follower.pathBuilder()
                .addPath(new BezierLine(beforeIntakeOne, afterIntakeOne))
                .setLinearHeadingInterpolation(beforeIntakeOne.getHeading(),afterIntakeOne.getHeading())
                .build();
        backToShootOne=follower.pathBuilder()
                .addPath(new BezierLine(afterIntakeOne, shootPose))
                .setLinearHeadingInterpolation(afterIntakeOne.getHeading(),shootPose.getHeading())
                .build();
        goToIntakeTwo=follower.pathBuilder()
                .addPath(new BezierLine(shootPose, beforeIntakeTwo))
                .setLinearHeadingInterpolation(shootPose.getHeading(), beforeIntakeTwo.getHeading())
                .build();
        intakeTwo=follower.pathBuilder()
                .addPath(new BezierLine(beforeIntakeTwo, afterIntakeTwo))
                .setLinearHeadingInterpolation(beforeIntakeTwo.getHeading(),afterIntakeTwo.getHeading())
                .build();
        backToShootTwo=follower.pathBuilder()
                .addPath(new BezierLine(afterIntakeTwo, shootPose))
                .setLinearHeadingInterpolation(afterIntakeTwo.getHeading(),shootPose.getHeading())
                .build();

        goToIntakeRec=follower.pathBuilder()
                .addPath(new BezierLine(shootPose, recordIntake))
                .setLinearHeadingInterpolation(shootPose.getHeading(),recordIntake.getHeading())
                .build();


    }


    public void statePathUpdate(){
        switch(pathState) {
            case START:
                launcher.setVelocity(2810);
                hood.setPosition(0.15);
                follower.setMaxPower(NORMAL_DRIVE_POWER);
                follower.followPath(firstShot);
                setPathState(kcBlueBack9.PathState.SHOOT);
                break;

            case SHOOT:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>2.5){
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(-1);
                }if (pathTimer.getElapsedTimeSeconds()>3.5){
                    follower.followPath(goToIntakeOne);
                    setPathState(kcBlueBack9.PathState.INTAKE);
                    tree.setPower(0);
                    theWheelOfTheOx.setPower(0);
                    launcher.setVelocity(0);
            }break;

            case INTAKE:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>0.5){
                    launcher.setVelocity(0);
                    tree.setPower(1);
                }if(pathTimer.getElapsedTimeSeconds()>1) {
                follower.followPath(intakeOne);
            }
                if(pathTimer.getElapsedTimeSeconds()>4){
                    tree.setPower(0);
                    if (pathTimer.getElapsedTimeSeconds()>5) {
                        launcher.setVelocity(2600);
                        follower.followPath(backToShootOne);
                        setPathState(kcBlueBack9.PathState.SHOOT1);
                    }
            } break;

            case SHOOT1:
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds()>2.5){
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(-1);
                }if (pathTimer.getElapsedTimeSeconds()>4){
                    follower.followPath(goToIntakeTwo);
                    setPathState(kcBlueBack9.PathState.INTAKE2);

                    theWheelOfTheOx.setPower(0);
                    launcher.setVelocity(0);
            }break;

            case INTAKE2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>1){
                    launcher.setVelocity(0);
                    tree.setPower(1);
            }if(pathTimer.getElapsedTimeSeconds()>1.5) {
                    follower.followPath(intakeTwo);
            }
                if (pathTimer.getElapsedTimeSeconds()>4) {
                    tree.setPower(0);
                    launcher.setVelocity(2610);
                    follower.followPath(backToShootTwo);
                    setPathState(kcBlueBack9.PathState.SHOOT2);
            }break;

            case SHOOT2:
                if(!follower.isBusy()&&pathTimer.getElapsedTimeSeconds()>1) {
                    tree.setPower(1);
                    theWheelOfTheOx.setPower(-1);
                    if (pathTimer.getElapsedTimeSeconds() > 3.5) {
                        follower.followPath(goToIntakeRec);
                        setPathState(kcBlueBack9.PathState.PARK);
                        theWheelOfTheOx.setPower(0);
                        launcher.setVelocity(0);
                    }
                }break;

            case PARK:
                if(!follower.isBusy()){
                    tree.setPower(0);

                }break;

        }
    }

    public void setPathState(kcBlueBack9.PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        // Reset flags when state changes to allow paths to be called again in new state
        shoot2Started = false;
        shoot3Started = false;
        shoot4Started = false;
        shoot5Started = false;
    }

    @Override
    public void init() {
        pathState = kcBlueBack9.PathState.START;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = ConstantsNewBot.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);


        tree = hardwareMap.get(DcMotor.class, "tree");
        theWheelOfTheOx = hardwareMap.get(DcMotor.class, "theWheelOfTheOx");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(0.0119);
        hood.scaleRange(0,0.0328);

        rotator = hardwareMap.get(DcMotor.class, "rotator");
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotatorStartPosition=0;
        rotator.setTargetPosition(rotatorStartPosition);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotator.setPower(1);

        theWheelOfTheOx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tree.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tree.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        follower.update();
        PoseStorage.savePose(follower.getPose());
        statePathUpdate();
        telemetry.addData("paths state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("jolly crusader velocity", launcher.getVelocity());
    }

    public double getDist(double tyDeg) {
        // Use corrected distance formula from TesterinoBlue
        double tyRad = Math.abs(Math.toRadians(tyDeg + limelightUpAngle));
        double dist = y / Math.tan(tyRad);
        double realDist = 0.55 * dist + 40.3; // Correction formula from TesterinoBlue
        return realDist;
    }
    public double calcVelocity(double dist) {
        // Use simpler linear formula from TesterinoBlue
        double velocity = 3.30933 * dist + 1507.01002;
        return velocity;
    }

    public void intake(double intakePower){
        tree.setPower(intakePower);
        if (!gamepad1.right_bumper) {
            theWheelOfTheOx.setPower(-0.3);
        }
    }


    public void adjustRotator(double tx) {
        double fracOfSemiCircum = Math.toRadians(tx) / Math.PI;
        int adjustment = (int) (fracOfSemiCircum * motor180Range);
        int newPosition = rotator.getCurrentPosition() + adjustment - offset;
        rotator.setTargetPosition(newPosition);
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
    public void updateLimelightAdjustments() {
        if (limelight != null) {
            LLResult ll = limelight.getLatestResult();
            if (ll != null && ll.isValid()) {
                // Limelight sees target - use current values
                txDeg = ll.getTx();
                tyDeg = ll.getTy();

                // Store valid values for fallback
                lastValidTx = txDeg;
                lastValidTy = tyDeg;

                // Calculate distance and store it
                double currentDistance = getDist(tyDeg);
                if (currentDistance > 0) {
                    lastValidDistance = currentDistance;
                    hasValidLimelightData = true;

                    // Adjust rotator based on horizontal offset
                    adjustRotator(txDeg);

                    // Update velocity and hood based on distance
                    launcher.setVelocity(calcVelocity(currentDistance));
                    adjustHoodBasedOnDistance(currentDistance);
                }
            } else {
                // Limelight doesn't see target - use last valid values if available
                if (hasValidLimelightData) {
                    // Use last known good values (from previous successful detection)
                    adjustRotator(lastValidTx);
                    if (lastValidDistance > 0) {
                        launcher.setVelocity(calcVelocity(lastValidDistance));
                        adjustHoodBasedOnDistance(lastValidDistance);
                    }
                }
                // If no valid data ever, do nothing (keep current settings from state initialization)
            }
        }
    }
    public void resetLimelightData() {
        hasValidLimelightData = false;
        lastValidTx = 0.0;
        lastValidTy = 0.0;
        lastValidDistance = 0.0;
    }

}







