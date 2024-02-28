package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double hoodAngleRad = 0.0;
        public double hoodGoalAngleRad = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodStatorCurrentAmps = 0.0;
        public double hoodSupplyCurrentAmps = 0.0;

        public double hoodVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public default void setHoodAngleRad(double angle) {}

    public default void setOverrideMode(boolean override) {}

    public default void setOverrideVolts(double volts) {}

    public default void home() {}

    public default void setPID(double p, double i, double d) {}

    public default void setBrakeMode(boolean brake) {}
}
