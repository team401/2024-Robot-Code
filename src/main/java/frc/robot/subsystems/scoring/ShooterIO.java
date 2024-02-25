package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterLeftVelocityRPM = 0.0;
        public double shooterLeftGoalVelocityRPM = 0.0;
        public double shooterLeftAppliedVolts = 0.0;
        public double shooterLeftStatorCurrentAmps = 0.0;
        public double shooterLeftSupplyCurrentAmps = 0.0;

        public double shooterRightVelocityRPM = 0.0;
        public double shooterRightGoalVelocityRPM = 0.0;
        public double shooterRightAppliedVolts = 0.0;
        public double shooterRightStatorCurrentAmps = 0.0;
        public double shooterRightSupplyCurrentAmps = 0.0;

        public double kickerAppliedVolts = 0.0;
        public double kickerStatorCurrentAmps = 0.0;

        public boolean bannerSensor = false;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setShooterVelocityRPM(double velocity) {}

    public default void setKickerVolts(double volts) {}

    public default void setOverrideMode(boolean override) {}

    public default void setOverrideVolts(double volts) {}

    public default void setPID(double p, double i, double d) {}

    public default void setMaxAcceleration(double maxAcceleration) {}

    public default void setMaxJerk(double maxJerk) {}

    public default void setFF(double kS, double kV, double kA) {}
}
