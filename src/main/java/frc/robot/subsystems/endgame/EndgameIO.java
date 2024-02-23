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

        public double position = 0.0;
        public double velocity = 0.0;
    }

    public default void updateInputs(EndgameIOInputs inputs) {}

    public default void setPID(double p, double i, double d) {}

    public default void setFF(double ff) {}

    public default void setOverrideMode(boolean override) {}

    public default void setOverrideVolts(double volts) {}

    public default void setPosition(double position) {}
}
