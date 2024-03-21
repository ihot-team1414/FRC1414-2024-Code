package frc.utils;

import java.util.TreeMap;

public class ShooterData {
    private static final TreeMap<Double, Double[]> shooterData = new TreeMap<Double, Double[]>();

    private static ShooterData instance;

    private ShooterData() {
        addSpeaker(2, 0.6, 9, 20);
        addSpeaker(2.1, 0.6, 8.75, 20);
        addSpeaker(2.25, 0.6, 8.5, 20);
        addSpeaker(2.35, 0.6, 8.25, 20);
        addSpeaker(2.55, 0.6, 8.0, 20);
        addSpeaker(2.65, 0.6, 7.75, 20);

        addSpeaker(2.75, 0.6, 7.6, 20);
        addSpeaker(2.9, 0.6, 7.45, 20);
        addSpeaker(3, 0.7, 7.25, 20);
        addSpeaker(3.15, 0.7, 7.175, 20);
        addSpeaker(3.25, 0.75, 7.125, 20);
        addSpeaker(3.35, 0.75, 7, 20);
        addSpeaker(3.5, 0.8, 6.85, 20);

        addSpeaker(6, 0.9, 5.4, 45);

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

    public double getShooterDutyCycle(double distance) {
        return getSpeakerEntry(distance)[0];
    }

    public double getShooterPosition(double distance) {
        return getSpeakerEntry(distance)[1];
    }

    public double getMinShotVelocity(double distance) {
        return getSpeakerEntry(distance)[2];
    }

    private void addSpeaker(double distance, double dutyCycle, double position, double minShotVelocity) {
        Double[] entry = { dutyCycle, position, minShotVelocity };
        shooterData.put(distance, entry);
    }
}
