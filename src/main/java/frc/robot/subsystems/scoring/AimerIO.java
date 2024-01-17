package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface AimerIO {
    @AutoLog
    public static class AimerIOInputs {
        public double aimAngleRad = 0.0;
        public double aimGoalAngleRad = 0.0;
        public double aimAppliedVolts = 0.0;
        public double aimCurrentAmps = 0.0;
    }

    public default void updateInputs(AimerIOInputs inputs) {}

    public default void setAimAngleRad(double angle) {}
}
