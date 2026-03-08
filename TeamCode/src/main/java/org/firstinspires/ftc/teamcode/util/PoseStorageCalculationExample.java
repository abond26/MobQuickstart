package org.firstinspires.ftc.teamcode.util;

/**
 * Example calculation showing how many ticks/degrees the rotator will move
 * when autonomous ends and teleop starts.
 * 
 * This is just for reference - the actual calculation happens in PoseStorage.
 */
public class PoseStorageCalculationExample {
    
    public static void main(String[] args) {
        // Example 1: close12red ending at park position
        System.out.println("=== close12red (Red Side) ===");
        calculateRotatorMovement(
            103.0, 84.0, Math.toRadians(46),  // Robot end position
            117.6, 130.0,                     // Red AprilTag
            910,                              // motor180Range
            28                                // offset
        );
        
        System.out.println("\n=== close12blue (Blue Side) ===");
        calculateRotatorMovement(
            35.0, 84.0, Math.toRadians(143),  // Robot end position
            24.4, 126.7,                      // Blue AprilTag
            910,                              // motor180Range
            28                                // offset
        );
    }
    
    public static void calculateRotatorMovement(
        double robotX, double robotY, double robotHeading,
        double aprilTagX, double aprilTagY,
        int motor180Range, int offset
    ) {
        // Calculate delta to AprilTag
        double deltaX = aprilTagX - robotX;
        double deltaY = aprilTagY - robotY;
        
        // Calculate absolute angle to AprilTag (in radians)
        double angleToTag = Math.atan2(deltaY, deltaX);
        
        // Calculate relative angle (how much to rotate from robot's current heading)
        double relativeAngle = angleToTag - robotHeading;
        
        // Normalize to [-PI, PI]
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;
        
        // Convert to degrees
        double angleDeg = Math.toDegrees(relativeAngle);
        
        // Convert angle to rotator ticks (semicircle: 180 degrees = motor180Range)
        double fracOfSemiCircum = Math.toRadians(angleDeg) / Math.PI;
        int rotatorTicks = (int) (fracOfSemiCircum * motor180Range);
        
        // Apply offset
        int finalTicks = rotatorTicks - offset;
        
        // Convert ticks to degrees (for reference)
        double degreesFromTicks = (finalTicks / (double) motor180Range) * 180.0;
        
        System.out.println("Robot Position: (" + robotX + ", " + robotY + ", " + Math.toDegrees(robotHeading) + "°)");
        System.out.println("AprilTag Position: (" + aprilTagX + ", " + aprilTagY + ")");
        System.out.println("Delta: (" + String.format("%.1f", deltaX) + ", " + String.format("%.1f", deltaY) + ")");
        System.out.println("Angle to Tag: " + String.format("%.1f", Math.toDegrees(angleToTag)) + "°");
        System.out.println("Relative Angle: " + String.format("%.1f", angleDeg) + "°");
        System.out.println("Rotator Ticks (before offset): " + rotatorTicks);
        System.out.println("Final Rotator Ticks (after -" + offset + " offset): " + finalTicks);
        System.out.println("Rotator Movement: " + String.format("%.1f", degreesFromTicks) + " degrees");
    }
}
