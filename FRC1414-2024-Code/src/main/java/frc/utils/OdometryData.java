package frc.utils;

import java.util.TreeMap;

import frc.robot.Constants.PivotConstants;

public class OdometryData {
    private static final TreeMap<Double, Double[]> shooterData = new TreeMap<Double, Double[]>();

    private static OdometryData instance;

    private OdometryData() {
        addOdometry(0, 9.2);
        addOdometry(1.46, 9); //5 inches from subwoofer
        addOdometry(1.68, 8.7); //15 inches
        addOdometry(1.85, 8); //20
        addOdometry(1.93, 7.7); //25
        addOdometry(2.05, 7.35); //30
        addOdometry(2.18, 7);
        addOdometry(2.31, 6.88);
        addOdometry(2.46, 6.7);
        addOdometry(2.6, 6.51);
        addOdometry(2.75, 6.32);
        addOdometry(2.85, 6.18);
        addOdometry(2.95, 6);
        addOdometry(3.12, 5.83);
        addOdometry(3.27, 5.68);
        addOdometry(3.35, 5.56);
        addOdometry(3.43, 5.53);
        addOdometry(3.52, 5.3);
        addOdometry(3.69, 5.1);
        addOdometry(3.75, 5.075);
        addOdometry(3.85, 5);
        addOdometry(3.88, 4.95);
        addOdometry(4.0, 4.86);
        addOdometry(4.1, 4.68);
        addOdometry(4.25, 4.56);
        addOdometry(4.33, 4.48);
        addOdometry(4.39, 4.42);
        addOdometry(4.54, 4.3);
        addOdometry(4.62, 4.25);
        addOdometry(4.78, 4.25);
        addOdometry(4.86, 4.24);
        addOdometry(5, 4.14);
        addOdometry(5.25, 4.04);



        
        // addOdometry(2, 8.8);
        // addOdometry(2.1, 8.3);
        // addOdometry(2.25, 8);
        // addOdometry(2.35, 7.75);
        // addOdometry(2.55, 7.5);
        // addOdometry(2.65, 7.2);
        // addOdometry(2.75, 7.0);
        // addOdometry(2.8, 6.9);
        // addOdometry(2.9, 6.75);
        // addOdometry(3, 6.25);
        // addOdometry(3.25, 6.17);
        // addOdometry(3.3, 6.05);
        // addOdometry(3.35, 6);
        // addOdometry(3.4, 5.90);
        // addOdometry(3.5, 5.85);
        // addOdometry(3.6, 5.75);
        // addOdometry(3.7, 5.65);
        // addOdometry(3.8, 5.6);
        // addOdometry(3.9, 5.55);
        // addOdometry(3.95, 5.5);
        // addOdometry(4, 5.4);
        // addOdometry(4.2, 5.4);
        // addOdometry(4.3, 5.3);
        // addOdometry(4.4, 5.25);
        // addOdometry(4.5, 5.25);
        // addOdometry(4.6, 5.2);
        // addOdometry(4.7, 5.2);
        // addOdometry(4.8, 5.15);
        // addOdometry(4.85, 5.25);
        // addOdometry(5.2, 4.8);
        // addOdometry(5.5, 4.6);
        // addOdometry(5.75, 4.6);
        // addOdometry(6, 4.45);
        // addOdometry(6.25, 4.4);
        // addOdometry(6.5, 4.3);
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
        Double[] empty = { 0.0, 0.0, 0.0 };

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
        double target = getInterpolatedSpeakerEntry(distance)[0];
        return target == 0 ? PivotConstants.kSpeakerShotPosition : target;
    }

    /*
    public double getShooterPosition(double distance, String sector) {
        if(sector.equals("AMP")) {
            double target = getInterpolatedSpeakerEntry(distance, 2)[0];
            return target == 0 ? PivotConstants.kSpeakerShotPosition : target;
        }
        else if(sector.equals("WEAK")) {
            double target = getInterpolatedSpeakerEntry(distance, 0)[0];
            return target == 0 ? PivotConstants.kSpeakerShotPosition : target;
        }
        else {
            return getShooterPosition(distance);
        }
    }*/

    /*
    private void addOdometry(double distance, double positionW, double positionC, double positionA) {
        Double[] entry = { positionW, positionC, positionA };
        shooterData.put(distance, entry);
    }*/

    private void addOdometry(double distance, double position){
        Double[] entry = {position};
        shooterData.put(distance, entry);
    }

}
