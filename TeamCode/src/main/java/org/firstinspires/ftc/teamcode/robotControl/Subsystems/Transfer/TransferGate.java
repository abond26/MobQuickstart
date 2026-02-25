package org.firstinspires.ftc.teamcode.robotControl.Subsystems.Transfer;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferGate implements TransferGConstants {

    private Servo blocker;

    public TransferGate(@NonNull HardwareMap hardwareMap) {
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(SCALE_RANGE_LOWER, SCALE_RANGE_UPPER);
    }

    public void block() {
        blocker.setPosition(BLOCK_POSITION);
    }

    public void open() {
        blocker.setPosition(OPEN_POSITION);
    }
}