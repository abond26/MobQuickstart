package org.firstinspires.ftc.teamcode.NewBotTele;

/**
 * Pure lookup table for distance to velocity conversion.
 *
 * NO FORMULAS - All values are empirically tuned and stored in the table.
 * Three distinct zones matching hood positions:
 * - Zone 1 (Close): < 140 inches
 * - Zone 2 (Mid): 140-200 inches
 * - Zone 3 (Far): >= 200 inches
 *
 * Velocity changes smoothly as distance changes through linear interpolation.
 * Tune the ShotTime array by testing actual distances - no formulas needed!
 */
public class ShotTimeLookupTable {

    // Distance thresholds matching hood zones
    private static final double CLOSE_THRESHOLD = 95;  // Zone 1: Close hood
    private static final double MID_THRESHOLD = 120;      // Zone 2: Mid hood, Zone 3: Far hood

    // Distance entries (in inches) - must be in ascending order
    // More entries around thresholds for smoother transitions
//    private static final double[] DISTANCES = {
//        // Zone 1: Close range (< 145) // hood position 1.
//        70,  79,   94,   109,   121, 139, // 1.
//        // Zone 2: Mid range (145-200) //hood 0.5
//        151.37,  165.53,  171.11, 155.6, 181.8, 193.1,
//        // Zone 3: Far range (>= 200)
//        210.5,  205.2,  221.5,
//    };
    private static final double[] DISTANCES = {
            86.0824, 69.0667,103.917,104.803,81.472

            // Zone 1: Close range (< 95) // hood position 1.

    };

    // Corresponding velocity values (in encoder ticks per second)
    // TUNE THESE VALUES EMPIRICALLY - No formulas, just test and adjust!
    // Start with reasonable values, then fine-tune based on actual shooting results
//    private static final double[] ShotTime = {
//        // Zone 1: Close range ShotTime (tune these for close shots)
//        1060, 1100, 1160, 1200, 1240, 1280,
//        // Zone 2: Mid range ShotTime (tune these for mid shots)
//        1320, 1380, 1400, 1360, 1460, 1500,
//        // Zone 3: Far range ShotTime (tune these for far shots)
//        1620, 1560, 1700,
//    };
    private static final double[] ShotTime = {
            1.01, 0.92,1.12, 1.13, 0.94

            // Zone 1: Close range ShotTime (tune these for close shots)

    };

    // Minimum and maximum distances in the table
    private static final double MIN_DISTANCE = DISTANCES[0];
    private static final double MAX_DISTANCE = DISTANCES[DISTANCES.length - 1];

    /**
     * Gets the velocity for a given distance using pure lookup table with linear interpolation.
     * NO FORMULAS - just looks up the value and interpolates between table entries.
     *
     * @param distance Distance to target in inches
     * @return Velocity in encoder ticks per second
     */
    public static double getTime(double distance) {
        // Clamp distance to table range
        if (distance <= MIN_DISTANCE) {
            return ShotTime[0];
        }
        if (distance >= MAX_DISTANCE) {
            return ShotTime[ShotTime.length - 1];
        }

        // Find the two table entries to interpolate between
        int index = 0;
        for (int i = 0; i < DISTANCES.length - 1; i++) {
            if (distance >= DISTANCES[i] && distance <= DISTANCES[i + 1]) {
                index = i;
                break;
            }
        }

        // Pure linear interpolation between the two entries (no formula)
        double dist1 = DISTANCES[index];
        double dist2 = DISTANCES[index + 1];
        double vel1 = ShotTime[index];
        double vel2 = ShotTime[index + 1];

        // Linear interpolation: smooth transition between table entries
        double t = (distance - dist1) / (dist2 - dist1);
        return vel1 + (vel2 - vel1) * t;
    }

    /**
     * Gets which zone (1, 2, or 3) a distance falls into.
     * Zone 1: Close (< 145), Zone 2: Mid (145-180), Zone 3: Far (>= 200)
     *
     * @param distance Distance to target in inches
     * @return Zone number (1, 2, or 3)
     */
    public static int getZone(double distance) {
        if (distance < CLOSE_THRESHOLD) {
            return 1; // Close zone (< 95)
        } else if (distance < MID_THRESHOLD) {
            return 2; // Mid zone (95-120)
        } else {
            return 3; // Far zone (>= 120)
        }
    }

    /**
     * Gets the minimum distance in the lookup table.
     */
    public static double getMinDistance() {
        return MIN_DISTANCE;
    }

    /**
     * Gets the maximum distance in the lookup table.
     */
    public static double getMaxDistance() {
        return MAX_DISTANCE;
    }

    /**
     * Gets the close/mid threshold (145 inches).
     */
    public static double getCloseThreshold() {
        return CLOSE_THRESHOLD;
    }

    /**
     * Gets the mid/far threshold (200 inches).
     */
    public static double getMidThreshold() {
        return 200.0; // Zone 3 starts at 200
    }
}
