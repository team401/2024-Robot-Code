package frc.robot.Utils;

public interface Tunable {
    public double getPosition();

    public double getVel();
    
    public void setVolts(double volts);

    public void stop();
}
