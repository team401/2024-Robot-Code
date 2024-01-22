package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterVelocityRPM = 0.0;
        public double shooterGoalVelocityRPM = 0.0;
        public double shooterAppliedVolts = 0.0;
        public double shooterCurrentAmps = 0.0;

        public double kickerAppliedVolts = 0.0;
        public double kickerCurrentAmps = 0.0;

        public boolean bannerSensor = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setShooterVelocityRPM(double velocity) {}

    public default void setKickerVolts(double volts) {}
}
