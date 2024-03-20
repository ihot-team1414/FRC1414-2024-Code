package frc.utils;

import java.util.TreeMap;

public class ShooterData {
    
    private static final TreeMap<Double, Double[]> shooterData = new TreeMap<Double, Double[]>();
    private static final TreeMap<Double, Double[]> safeData = new TreeMap<Double, Double[]>();

    private static ShooterData instance;

    private ShooterData(){

        // Add data to the TreeMap (current values are filler)
        addSpeaker(43.65, 0.0, 10);
        addSpeaker(80.0, 1.0, 8);
        addSpeaker(2.0, 2.0, 2.0);
        addSpeaker(3.0, 3.0, 3.0);
        addSpeaker(4.0, 4.0, 4.0);
        addSpeaker(5.0, 5.0, 5.0);
        addSpeaker(6.0, 6.0, 6.0);
        addSpeaker(7.0, 7.0, 7.0);
        addSpeaker(8.0, 8.0, 8.0);
        addSpeaker(9.0, 9.0, 9.0);

        // Add data to the TreeMap (current values are filler)
        addSafe(0.0, 0.0, 0.0);
        addSafe(1.0, 1.0, 1.0);
        addSafe(2.0, 2.0, 2.0);
        addSafe(3.0, 3.0, 3.0);
        addSafe(4.0, 4.0, 4.0);
        addSafe(5.0, 5.0, 5.0);
        addSafe(6.0, 6.0, 6.0);
        addSafe(7.0, 7.0, 7.0);
        addSafe(8.0, 8.0, 8.0);
        addSafe(9.0, 9.0, 9.0);
        
    }

    public static synchronized ShooterData getInstance(){
        if (instance == null) {
            instance = new ShooterData();
        }
        return instance;
    }

    public Double[] getSpeakerEntry(double distance) {
        Double[] empty = { 0.0, 0.0 };

        try {

            //Get the closest distance to the given distance
            Double ceiling = shooterData.ceilingKey(distance);
            Double floor = shooterData.floorKey(distance);

            if (ceiling != null && floor != null) {

                //Return the closest distance to the given distance
                return Math.abs(ceiling - distance) < Math.abs(floor - distance) ? shooterData.get(ceiling) : shooterData.get(floor);
            } else if (ceiling != null) {
                return shooterData.get(ceiling);
            } else if (floor != null) {
                return shooterData.get(floor);
            } else {
                return empty;
            }
        } catch(Exception e) {
            return empty;
        }
    }

        public Double[] getSafeEntry(double distance) {
        Double[] empty = { 0.0, 0.0 };

        try {

            //Get the closest distance to the given distance
            Double ceiling = safeData.ceilingKey(distance);
            Double floor = safeData.floorKey(distance);

            if (ceiling != null && floor != null) {

                //Return the closest distance to the given distance
                return Math.abs(ceiling - distance) < Math.abs(floor - distance) ? safeData.get(ceiling) : safeData.get(floor);
            } else if (ceiling != null) {
                return safeData.get(ceiling);
            } else if (floor != null) {
                return safeData.get(floor);
            } else {
                return empty;
            }
        } catch(Exception e) {
            return empty;
        }
    }

    public double getSafeSpeed(double distance){
        return getSpeakerEntry(distance)[0];
    }

    public double getSafeAngle(double distance){
        return getSpeakerEntry(distance)[1];
    }

    public double getShooterSpeed(double distance){
        return getSpeakerEntry(distance)[0];
    }

    public double getShooterAngle(double distance){
        return getSpeakerEntry(distance)[1];
    }

    private void addSpeaker(double distance, double speed, double angle) {
        Double[] speedAngle = { speed, angle };
        shooterData.put(distance, speedAngle);
    }

    private void addSafe(double distance, double speed, double angle) {
        Double[] speedAngle = { speed, angle };
        safeData.put(distance, speedAngle);
    }

}

