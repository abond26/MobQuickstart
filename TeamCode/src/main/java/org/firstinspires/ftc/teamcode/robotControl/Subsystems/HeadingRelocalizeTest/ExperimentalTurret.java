package org.firstinspires.ftc.teamcode.robotControl.Subsystems.HeadingRelocalizeTest;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.robotControl.Subsystems.Turret.Turret;

/**
 * Experimental version of Turret.
 * Includes software encoder offsets to handle gear slippage.
 */
public class ExperimentalTurret extends Turret {
    private int rotatorOffset = 0;

    public ExperimentalTurret(@NonNull HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void setRotatorPos(int ticks) {
        // We override the parent to apply the offset
        super.setRotatorPos(ticks - rotatorOffset);
    }

    @Override
    public int getRotatorPos() {
        // Note: This relies on the parent's getRotatorPos returning the internal raw position
        // Since we want to use the parent's sensor reading plus our offset.
        // However, Turret.java's getRotatorPos() just calls rotator.getCurrentPosition().
        // So we just call the super and add the offset.
        return super.getRotatorPos() + rotatorOffset;
    }

    /**
     * Resets the logical encoder position to a specific value.
     * Use this for "resyncing" if the gears slip.
     */
    public void resetRotatorEncoder(int currentPhysicalTicks) {
        int rawPos = super.getRotatorPos() - rotatorOffset; // Subtract current offset to get raw
        rotatorOffset = currentPhysicalTicks - rawPos;
        
        // No need to physically reset the motor, just the software offset.
    }
}
