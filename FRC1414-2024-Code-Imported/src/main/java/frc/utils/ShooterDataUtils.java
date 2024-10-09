package frc.utils;

import java.util.TreeMap;

import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterDataUtils {
    private static double interpolate(double a, double b, double frac) {
        return a + (b - a) * frac;
    }

    public static ShooterEntry getInterpolatedEntry(TreeMap<Double, ShooterEntry> data, double distance) {
        if (data.get(distance) != null) {
            return data.get(distance);
        }
        ShooterEntry empty = new ShooterEntry(PivotConstants.kSubwooferShotPosition,
                ShooterConstants.kSubwooferShotVelocity);

        try {
            // Get the closest distance to the given distance

            Double ceiling = data.ceilingKey(distance);
            Double floor = data.floorKey(distance);

            if (ceiling != null && floor != null) {

                // Return the closest distance to the given distance
                double frac = (distance - data.floorKey(distance))
                        / (data.ceilingKey(distance) - data.floorKey(distance));

                ShooterEntry result = new ShooterEntry(
                        interpolate(data.get(floor).getPosition(), data.get(ceiling).getPosition(), frac),
                        data.floorEntry(floor).getValue().getMinShotVelocity());
                return result;
            } else if (ceiling != null) {
                return data.get(ceiling);
            } else if (floor != null) {
                return data.get(floor);
            } else {
                return empty;
            }
        } catch (Exception e) {
            return empty;
        }
    }
}
