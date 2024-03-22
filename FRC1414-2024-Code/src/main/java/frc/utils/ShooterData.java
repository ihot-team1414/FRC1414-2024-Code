package frc.utils;

import java.util.TreeMap;

public class ShooterData {
    private static final TreeMap<Double, Double[]> shooterData = new TreeMap<Double, Double[]>();

    private static ShooterData instance;

    private ShooterData() {
        addSpeaker(1.75, 0.6, 10, 20);
        addSpeaker(2, 0.6, 9.5, 20);
        addSpeaker(2.1, 0.6, 9, 20);
        addSpeaker(2.25, 0.6, 8.5, 20);
        addSpeaker(2.35, 0.6, 8.25, 20);
        addSpeaker(2.55, 0.6, 8, 20);
        addSpeaker(2.65, 0.6, 7.75, 20);
        addSpeaker(2.75, 0.6, 7.6, 20);
        addSpeaker(2.8, 0.6, 7.75, 20);
        addSpeaker(2.9, 0.6, 7.45, 20);

        addSpeaker(3, 0.7, 6.75, 20);
        addSpeaker(3.15, 0.7, 6.75, 20);
        addSpeaker(3.2, 0.7, 6.75, 20);
        addSpeaker(3.25, 0.75, 6.70, 20);
        addSpeaker(3.3, 0.8, 6.6, 20);
        addSpeaker(3.35, 0.75, 6.50, 20);
        addSpeaker(3.4, 0.8, 6.40, 20);
        addSpeaker(3.5, 0.8, 6.25, 20);
        addSpeaker(3.6, 0.8, 6.10, 20);
        addSpeaker(3.65, 0.8, 5.98, 20);
        addSpeaker(3.7, 0.8, 5.95, 20);
        addSpeaker(3.8, 0.8, 5.9, 20);
        addSpeaker(3.9, 0.8, 5.85, 20);
        addSpeaker(3.95, 0.8, 5.85, 20);
        addSpeaker(4, 0.8, 5.6, 20);
        addSpeaker(5, 0.8, 5.2, 40);

        addSpeaker(6, 0.9, 5.375, 45);

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
