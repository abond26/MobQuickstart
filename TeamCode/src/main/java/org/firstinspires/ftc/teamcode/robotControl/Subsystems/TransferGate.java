package org.firstinspires.ftc.teamcode.robotControl.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TransferGate {

    private Servo blocker;

    private final double BLOCK_POSITION = 0;
    private final double OPEN_POSITION = 1;

    public TransferGate(HardwareMap hardwareMap) {
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.scaleRange(0, 0.4);
    }

    public void block() {
        blocker.setPosition(BLOCK_POSITION);
    }

    public void open() {
        blocker.setPosition(OPEN_POSITION);
    }
}