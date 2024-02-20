package frc.robot.subsystems.endgame;

import org.littletonrobotics.junction.AutoLog;

public interface EndgameIO {

    @AutoLog
    public static class EndgameIOInputs {
        public double endgameLeftAppliedVolts = 0.0;
        public double endgameLeftCurrentAmps = 0.0;

        public double endgameRightAppliedVolts = 0.0;
        public double endgameRightCurrentAmps = 0.0;

        public double position = 0.0;
        public double velocity = 0.0;
    }

    public default void updateInputs(EndgameIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setBrakeMode(boolean brake) {}
    ;
}
