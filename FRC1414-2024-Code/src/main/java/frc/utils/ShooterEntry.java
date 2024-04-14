package frc.utils;

public class ShooterEntry {
    private double position;
    private double minShotVelocity;

    public ShooterEntry(double position, double minShotVelocity) {
        this.position = position;
        this.minShotVelocity = minShotVelocity;
    }

    public double getPosition() {
        return position;
    }

    public double getMinShotVelocity() {
        return minShotVelocity;
    }
}
