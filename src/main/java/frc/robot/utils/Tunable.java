package frc.robot.utils;

// Interface for subsystems to be tuned
public interface Tunable {
    public double getPosition();

    public double getVel();

    public void setVolts(double volts);

    public void stop();
}
