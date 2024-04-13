package frc.utils;

import java.util.TreeMap;
import frc.robot.Constants.PivotConstants;

public class OdometryData {
    private static final TreeMap<Double, Double[]> shooterData = new TreeMap<Double, Double[]>();

    private static OdometryData instance;

    private OdometryData() {
        addOdometry(0, 0);
    }

    public static synchronized OdometryData getInstance() {
        if (instance == null) {
            instance = new OdometryData();
        }
        return instance;
    }

    private double interpolate(double a, double b, double frac) {
        return a + (b - a) * frac;
    }

    public Double[] getInterpolatedSpeakerEntry(double distance) {
        Double[] empty = { 0.0, 0.0 };

        if (shooterData.get(distance) != null) {
            return shooterData.get(distance);
        }

        try {

            // Get the closest distance to the given distance
            Double ceiling = shooterData.ceilingKey(distance);
            Double floor = shooterData.floorKey(distance);

            if (ceiling != null && floor != null) {

                // Return the closest distance to the given distance
                double frac = (distance - shooterData.floorKey(distance))
                        / (shooterData.ceilingKey(distance) - shooterData.floorKey(distance));

                Double[] result = { interpolate(shooterData.get(floor)[0], shooterData.get(ceiling)[0], frac) };
                return result;
            } else if (ceiling != null) {
                return shooterData.get(ceiling);
            } else if (floor != null) {
                return shooterData.get(floor);
            } else {
                return empty;
            }
        } catch (Exception e) {
            return empty;
        }
    }

    public double getShooterPosition(double distance) {
        double target = getInterpolatedSpeakerEntry(distance)[1];
        return target == 0 ? PivotConstants.kSpeakerShotPosition : target;
    }

    public double getShooterPosition(double distance, String sector) {
        if(sector.equals("AMP")) {
            double target = getInterpolatedSpeakerEntry(distance)[2];
            return target == 0 ? PivotConstants.kSpeakerShotPosition : target;
        }
        else if(sector.equals("WEAK")) {
            double target = getInterpolatedSpeakerEntry(distance)[0];
            return target == 0 ? PivotConstants.kSpeakerShotPosition : target;
        }
        else {
            return getShooterPosition(distance);
        }
    }

    private void addOdometry(double distance, double positionW, double positionC, double positionA) {
        Double[] entry = { positionW, positionC, positionA };
        shooterData.put(distance, entry);
    }

    private void addOdometry(double distance, double position){
        addOdometry(distance, position, position, position);
    }

}
