package org.firstinspires.ftc.teamcode.NewBotAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robotControl.RobotActions;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.ConstantsNewBot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import android.util.Log;

/**
 * Drives to (72, 72), then spins forever. TJ phrase is repeated every {@code loop()} (log + nested
 * loops) until the op mode is stopped.
 */
@Autonomous(name = "Auton For TJ ;)", group = "auto", preselectTeleOp = "AmazingBotRed")
public class NoRampAuton extends OpMode {

    private static final String TAG = "NoRampAuton";
    private static final String TJ_AUTON_MSG =
            "TJ said he didn't want auton basically you can ask him why";

    private static final int PIPELINENUM = 0;
    private static final double NORMAL_DRIVE_POWER = 1.0;
    private static final Pose CENTER_POSE = new Pose(72, 72, 0);
    private static final double SPIN_ROT_POWER = -0.45;
    /** Repetitions per {@code loop()} into log / Logcat (nested while × while). */
    private static final int PHRASE_OUTER = 100;
    private static final int PHRASE_INNER = 100;

    private Follower follower;
    private Robot robot;
    private PathChain goToCenter;
    private Timer opModeTimer;
    private long phraseRepeatCount;

    private enum Phase {
        DRIVE_TO_CENTER,
        SPIN
    }

    private Phase phase = Phase.DRIVE_TO_CENTER;
    private boolean driveStarted;
    private boolean teleopSpinPrimed;

    private final Pose startPose =
            new Pose(112.21723544631305, 136.97551020408164, Math.toRadians(-90));

    private void buildPaths() {
        goToCenter =
                follower.pathBuilder()
                        .addPath(new BezierLine(startPose, CENTER_POSE))
                        .setLinearHeadingInterpolation(
                                startPose.getHeading(), CENTER_POSE.getHeading())
                        .build();
    }

    /** Nested whiles: spam phrase to DS log + Logcat. */
    private void repeatPhraseNested() {
        int outer = 0;
        while (outer < PHRASE_OUTER) {
            int inner = 0;
            while (inner < PHRASE_INNER) {
                Log.i(TAG, TJ_AUTON_MSG);
                telemetry.log().add(TJ_AUTON_MSG);
                phraseRepeatCount++;
                inner++;
            }
            outer++;
        }
    }

    @Override
    public void init() {
        follower = ConstantsNewBot.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        robot = new Robot(hardwareMap, startPose, PIPELINENUM);
        new RobotActions(
                robot.chassisLocal,
                robot.vision,
                robot.turret,
                robot.gate,
                robot.intake);
        robot.intake.down();
        robot.gate.block();

        opModeTimer = new Timer();

        telemetry.addLine("NoRampAuton: init");
        telemetry.update();
    }

    @Override
    public void start() {
        phraseRepeatCount = 0;
        repeatPhraseNested();
        telemetry.update();

        opModeTimer.resetTimer();
        phase = Phase.DRIVE_TO_CENTER;
        driveStarted = false;
        teleopSpinPrimed = false;
    }

    @Override
    public void loop() {
        repeatPhraseNested();

        telemetry.addData("TJ note", TJ_AUTON_MSG);
        telemetry.addData("TJ repeats (approx)", phraseRepeatCount);

        switch (phase) {
            case DRIVE_TO_CENTER:
                if (!driveStarted) {
                    follower.setMaxPower(NORMAL_DRIVE_POWER);
                    follower.followPath(goToCenter);
                    driveStarted = true;
                }
                if (!follower.isBusy() || isNear(CENTER_POSE, 4.0)) {
                    phase = Phase.SPIN;
                }
                break;
            case SPIN:
                if (!teleopSpinPrimed) {
                    follower.startTeleopDrive();
                    teleopSpinPrimed = true;
                }
                follower.setTeleOpDrive(0, 0, SPIN_ROT_POWER, true);
                break;
        }

        telemetry.update();
    }

    private boolean isNear(Pose target, double thresholdInches) {
        Pose p = follower.getPose();
        double dx = p.getX() - target.getX();
        double dy = p.getY() - target.getY();
        return Math.sqrt(dx * dx + dy * dy) < thresholdInches;
    }
}
