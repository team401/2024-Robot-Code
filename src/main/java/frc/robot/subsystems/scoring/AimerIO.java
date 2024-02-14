package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface AimerIO {
    @AutoLog
    public static class AimerIOInputs {
        public double aimAngleRad = 0.0;
        public double aimGoalAngleRad = 0.0;
        public double aimAppliedVolts = 0.0;
        public double aimCurrentAmps = 0.0;

        public double aimVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(AimerIOInputs inputs) {}

    public default void setAimAngleRad(double angle, boolean newProfile) {}

    public default void controlAimAngleRad() {}

    public default void setAngleClampsRad(double min, double max) {}

    public default void setOverrideMode(boolean override) {}

    public default void setOverrideVolts(double volts) {}

    public default void setPID(double p, double i, double d) {}
}
