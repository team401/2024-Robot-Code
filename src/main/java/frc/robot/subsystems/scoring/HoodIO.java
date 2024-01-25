package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public double hoodAngleRad = 0.0;
        public double hoodGoalAngleRad = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodCurrentAmps = 0.0;
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public default void setHoodAngleRad(double angle) {}
}
