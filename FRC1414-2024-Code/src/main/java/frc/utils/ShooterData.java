package frc.utils;

import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;

public class ShooterData {
    private static final TreeMap<Double, Double[]> shooterData = new TreeMap<Double, Double[]>();

    private static ShooterData instance;

    private ShooterData() {
        addSpeaker(0, 6);
        addSpeaker(1.75, 9.9);
        addSpeaker(2, 8.8);
        addSpeaker(2.1, 8.3);
        addSpeaker(2.25, 8);
        addSpeaker(2.35, 7.75);
        addSpeaker(2.55, 7.5);
        addSpeaker(2.65, 7.2);
        addSpeaker(2.75, 7.0);
        addSpeaker(2.8, 6.9);
        addSpeaker(2.9, 6.75);
        addSpeaker(3, 6.25);
        addSpeaker(3.25, 6.17);
        addSpeaker(3.3, 6.05);
        addSpeaker(3.35, 6.95);
        addSpeaker(3.4, 5.90);
        addSpeaker(3.5, 5.75);
        addSpeaker(3.6, 5.7);
        addSpeaker(3.7, 5.65);
        addSpeaker(3.8, 5.55);
        addSpeaker(3.9, 5.5);
        addSpeaker(3.95, 5.45);
        addSpeaker(4, 5.4);
        addSpeaker(4.2, 5.4);
        addSpeaker(4.3, 5.3);
        addSpeaker(4.4, 5.25);
        addSpeaker(4.5, 5.05);
        addSpeaker(4.6, 5);
        addSpeaker(4.7, 4.95);
        addSpeaker(4.8, 4.9);
        addSpeaker(5.2, 4.8);
        addSpeaker(5.5, 4.6);
        addSpeaker(5.75, 4.6);
        addSpeaker(6, 4.45);
        addSpeaker(6.25, 4.4);
        addSpeaker(6.5, 4.3);

    }

    public static synchronized ShooterData getInstance() {
        if (instance == null) {
            instance = new ShooterData();
        }
        return instance;
    }

    public Double[] getSpeakerEntry(double distance) {
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
                return Math.abs(ceiling - distance) < Math.abs(floor - distance) ? shooterData.get(ceiling)
                        : shooterData.get(floor);
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
        double target = getInterpolatedSpeakerEntry(distance)[0];
        SmartDashboard.putNumber("Pivot Target", target);
        return target == 0 ? PivotConstants.kSpeakerShotPosition : target - 0.5;
    }

    public double getShooterPosition(Optional<Double> distance) {
        return distance.isPresent() ? getShooterPosition(distance.get()) : PivotConstants.kSpeakerShotPosition;
    }

    private void addSpeaker(double distance, double position) {
        Double[] entry = { position };
        shooterData.put(distance, entry);
    }

}
