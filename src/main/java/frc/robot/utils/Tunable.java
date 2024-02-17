package frc.robot.utils;

/**
 * Interface for subsystems to be tuned.
 *
 * <p>Slots are used to differentiate between different motor-groups/io-elements in the same
 * subsystem
 */
public interface Tunable {
    public double getPosition(int slot);

    public double getVelocity(int slot);

    public double getConversionFactor(int slot);

    public void setVolts(double volts, int slot);

    public void setPID(double p, double i, double d, int slot);

    public void setFF(double kS, double kV, double kA, double kG, int slot);

    public void setMaxProfileVelocity(double maxVelocity, int slot);

    public void setMaxProfileAcceleration(double maxAcceleration, int slot);

    public void runToPosition(double position, int slot);
}
