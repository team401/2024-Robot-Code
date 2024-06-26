package frc.robot.subsystems.endgame;

import org.littletonrobotics.junction.AutoLog;

public interface EndgameIO {
    @AutoLog
    public static class EndgameIOInputs {
        public double endgameLeftAppliedVolts = 0.0;

        public double endgameLeftStatorCurrentAmps = 0.0;
        public double endgameLeftSupplyCurrentAmps = 0.0;

        public double endgameRightAppliedVolts = 0.0;

        public double endgameRightStatorCurrentAmps = 0.0;
        public double endgameRightSupplyCurrentAmps = 0.0;

        public double overrideVolts = 0.0;

        public double position = 0.0;
        public double velocity = 0.0;

        public double profileTargetPosition = 0.0;
        public double finalTargetPosition = 0.0;

        public double profileTimerTime = 0.0;
    }

    public default void updateInputs(EndgameIOInputs inputs) {}

    public default void setPID(double p, double i, double d) {}

    public default void setFF(double ff) {}

    public default void setClimbing(boolean climbing) {}

    public default void setOverrideMode(boolean override) {}

    public default void setOverrideVolts(double volts) {}

    public default void setBrakeMode(boolean brake) {}

    public default void setPosition(double position) {}

    public default void setPositionTuning(double position) {}

    public default void setMaxProfile(double maxVelocity, double maxAcceleration) {}
}
