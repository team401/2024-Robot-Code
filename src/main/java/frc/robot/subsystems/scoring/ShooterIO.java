package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterVelRPM = 0.0;
        public double shooterGoalVelRPM = 0.0;
        public double shooterAppliedVolts = 0.0;
        public double shooterCurrentAmps = 0.0;

        public double kickerAppliedVolts = 0.0;
        public double kickerCurrentAmps = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setShooterVelRPM(double vel) {}

    public default void setKickerVolts(double volts) {}
}
