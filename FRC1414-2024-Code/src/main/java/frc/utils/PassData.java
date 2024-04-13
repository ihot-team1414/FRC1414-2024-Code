package frc.utils;

import java.util.TreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PassData {
    private static final TreeMap<Double, Double[]> passData = new TreeMap<Double, Double[]>();

    private static PassData instance;

    private PassData() {
        addPass(5, 6);

    }

    public static synchronized PassData getInstance() {
        if (instance == null) {
            instance = new PassData();
        }
        return instance;
    }

    private double interpolate(double a, double b, double frac) {
        return a + (b - a) * frac;
    }

    public Double[] getInterpolatedPassEntry(double distance) {
        Double[] empty = { 0.0, 0.0 };

        if (passData.get(distance) != null) {
            return passData.get(distance);
        }

        try {

            // Get the closest distance to the given distance
            Double ceiling = passData.ceilingKey(distance);
            Double floor = passData.floorKey(distance);

            if (ceiling != null && floor != null) {

                // Return the closest distance to the given distance
                double frac = (distance - passData.floorKey(distance))
                        / (passData.ceilingKey(distance) - passData.floorKey(distance));

                Double[] result = { interpolate(passData.get(floor)[0], passData.get(ceiling)[0], frac) };
                return result;
            } else if (ceiling != null) {
                return passData.get(ceiling);
            } else if (floor != null) {
                return passData.get(floor);
            } else {
                return empty;
            }
        } catch (Exception e) {
            return empty;
        }
    }

    public double getShooterPosition(double distance) {
        double target = getInterpolatedPassEntry(distance)[0];
        SmartDashboard.putNumber("Pivot Target", target);
        return target;
    }

    private void addPass(double distance, double position) {
        Double[] entry = { position };
        passData.put(distance, entry);
    }

}
