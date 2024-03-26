package frc.utils;

import java.util.Optional;
import java.util.TreeMap;

import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterData {
    private static final TreeMap<Double, Double[]> shooterData = new TreeMap<Double, Double[]>();

    private static ShooterData instance;

    private ShooterData() {
        addSpeaker(0, 6);
        addSpeaker(1.75, 10);
        addSpeaker(2, 9.5);
        addSpeaker(2.1, 9);
        addSpeaker(2.25, 8.5);
        addSpeaker(2.35, 8.25);
        addSpeaker(2.55, 8);
        addSpeaker(2.65, 7.75);
        addSpeaker(2.75, 7.6);
        addSpeaker(2.8, 7.75);
        addSpeaker(2.9, 7.45);
        addSpeaker(3, 6.75);
        addSpeaker(3.25, 6.67);
        addSpeaker(3.3, 6.6);
        addSpeaker(3.35, 6.50);
        addSpeaker(3.4, 6.40);
        addSpeaker(3.5, 6.25);
        addSpeaker(3.6, 6.10);
        addSpeaker(3.65, 5.98);
        addSpeaker(3.7, 5.95);
        addSpeaker(3.8, 5.9);
        addSpeaker(3.9, 5.85);
        addSpeaker(3.95, 5.85);
        addSpeaker(4, 5.6);
        addSpeaker(4.3, 5.55);
        addSpeaker(4.4, 5.45);
        addSpeaker(4.8, 5.1);
        addSpeaker(6, 5.375);

    }

    public static synchronized ShooterData getInstance() {
        if (instance == null) {
            instance = new ShooterData();
        }
        return instance;
    }

    public Double[] getSpeakerEntry(double distance) {
        Double[] empty = { 0.0, 0.0 };

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

    public double getShooterPosition(double distance) {
        return getSpeakerEntry(distance)[0];
    }

    public double getShooterPosition(Optional<Double> distance) {
        return distance.isPresent() ? getShooterPosition(distance.get()) : PivotConstants.kSpeakerShotPosition;
    }

    private void addSpeaker(double distance, double position) {
        Double[] entry = { position };
        shooterData.put(distance, entry);
    }

}
