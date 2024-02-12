package frc.robot.utils;

// Interface for subsystems to be tuned
public interface Tunable {
    public double getPosition();

    public double getPosition(int slot);

    public double getVelocity();

    public double getVelocity(int slot);

    public void setVolts(double volts);

    public void setVolts(double volts, int slot);

    public void setPID(double p, double i, double d);

    public void setPID(double p, double i, double d, int slot);
}
