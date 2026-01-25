# PoseStorage - Pass Position from Autonomous to TeleOp

This utility allows you to automatically pass the robot's final position from autonomous mode to teleop mode, so teleop starts exactly where autonomous ended.

## Problem

- Different autonomous modes end in different positions
- Teleop always starts at `(0, 0, 0)` by default
- This causes incorrect localization in teleop if autonomous ended elsewhere

## Solution


Use `PoseStorage` to save the final pose at the end of autonomous and load it at the start of teleop.

## How to Use

### Step 1: In Your Autonomous Mode

**IMPORTANT**: Save the pose continuously in your `loop()` method, not just at the end!
This ensures the pose is saved even if autonomous ends early due to error, timeout, or being stopped.

```java
import org.firstinspires.ftc.teamcode.util.PoseStorage;

// In your loop() method - save pose continuously:
@Override
public void loop() {
    follower.update();
    statePathUpdate();
    
    // Save pose continuously (so it's saved even if autonomous ends early)
    PoseStorage.savePose(follower.getPose());
    
    // Rest of your loop code...
    telemetry.addData("x", follower.getPose().getX());
    // etc.
}
```

**Alternative**: If you prefer, you can save at key waypoints instead:
```java
case done:
    PoseStorage.savePose(follower.getPose());  // Final save
    break;
case parklol:
    PoseStorage.savePose(follower.getPose());  // Save when parking
    // ... rest of code
    break;
```

**Example for `red15close.java`:**

```java
case done:
    // Save final pose for teleop
    PoseStorage.savePose(follower.getPose());
    telemetry.addData("Saved pose", "X: %.2f, Y: %.2f, Heading: %.2f", 
        follower.getPose().getX(), 
        follower.getPose().getY(), 
        Math.toDegrees(follower.getPose().getHeading()));
    break;
```

### Step 2: In Your TeleOp Mode

Modify your teleop initialization to load the saved pose:

```java
import org.firstinspires.ftc.teamcode.util.PoseStorage;

public void runOpMode() throws InterruptedException {
    follower = Constants.createFollower(hardwareMap);
    
    // Load saved pose from autonomous, or use default if none saved
    Pose startPose = PoseStorage.loadPose(new Pose(0, 0, 0));
    follower.setStartingPose(startPose);
    
    // Rest of initialization...
}
```

**Example for `TestingNewAdjustment.java`:**

Change this:
```java
private final Pose startPose = new Pose(0, 0, 0);
// ...
follower.setStartingPose(startPose);
```

To this:
```java
// Remove the final keyword and initialize in runOpMode
private Pose startPose;

public void runOpMode() throws InterruptedException {
    follower = Constants.createFollower(hardwareMap);
    
    // Load saved pose from autonomous, or default to (0, 0, 0)
    startPose = PoseStorage.loadPose(new Pose(0, 0, 0));
    follower.setStartingPose(startPose);
    
    // Optional: Show in telemetry
    if (PoseStorage.hasSavedPose()) {
        telemetry.addData("Starting from", "Autonomous end position");
    } else {
        telemetry.addData("Starting from", "Default position");
    }
    
    // Rest of initialization...
}
```

## Benefits

✅ **Automatic**: No need to manually set different starting positions for each autonomous  
✅ **Accurate**: Teleop always starts exactly where autonomous ended  
✅ **Safe**: Falls back to default `(0, 0, 0)` if no pose was saved  
✅ **Simple**: Just 2 lines of code in each mode  

## Important Notes

1. **Save continuously**: Save the pose in your `loop()` method, not just at the end. This ensures the pose is saved even if autonomous ends early due to:
   - Timeout (30 seconds)
   - Error/crash
   - Being manually stopped
   - Early termination

2. **Load early**: Load the pose before calling `follower.setStartingPose()`

3. **Default fallback**: Always provide a default pose in case autonomous wasn't run

4. **Testing**: If testing teleop without running autonomous first, it will use the default pose

5. **Performance**: Saving the pose is very lightweight (just storing 3 doubles), so calling it every loop cycle is fine

## Example: Complete Flow

**Autonomous (`red15close.java`):**
```java
@Override
public void loop() {
    follower.update();
    statePathUpdate();
    
    // Save pose continuously (handles early termination)
    PoseStorage.savePose(follower.getPose());
    
    // Rest of your telemetry and code...
    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.update();
}
```

**TeleOp (`TestingNewAdjustment.java`):**
```java
public void runOpMode() throws InterruptedException {
    follower = Constants.createFollower(hardwareMap);
    Pose startPose = PoseStorage.loadPose(new Pose(0, 0, 0));
    follower.setStartingPose(startPose);
    // ... rest of code
}
```

That's it! Now teleop will always start exactly where autonomous ended, regardless of which autonomous mode you ran.
