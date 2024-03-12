package frc.utils;

import java.util.TreeMap;

public class ShooterData {
    
    private static final TreeMap<Double, Double[]> shooterData = new TreeMap<Double, Double[]>();
    private static ShooterData instance;

    private ShooterData(){

        // Add data to the TreeMap (current values are filler)
        add(0.0, 0.0, 0.0);
        add(1.0, 1.0, 1.0);
        add(2.0, 2.0, 2.0);
        add(3.0, 3.0, 3.0);
        add(4.0, 4.0, 4.0);
        add(5.0, 5.0, 5.0);
        add(6.0, 6.0, 6.0);
        add(7.0, 7.0, 7.0);
        add(8.0, 8.0, 8.0);
        add(9.0, 9.0, 9.0);
        
    }

    public static synchronized ShooterData getInstance(){
        if (instance == null) {
            instance = new ShooterData();
        }
        return instance;
    }

    public Double[] getEntry(double distance) {
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

    public double getShooterSpeed(double distance){
        return getEntry(distance)[0];
    }

    public double getShooterAngle(double distance){
        return getEntry(distance)[1];
    }

    private void add(double distance, double speed, double angle) {
        Double[] speedAngle = { speed, angle };

        shooterData.put(distance, speedAngle);
    }

}

