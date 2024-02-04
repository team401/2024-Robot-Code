package frc.robot.subsystems.endgame;

import org.littletonrobotics.junction.AutoLog;

public interface EndgameIO {

    @AutoLog
    public static class EndgameIOInputs {
        public double endgameLeftMotorCurrent = 0.0;
        public double endgameRightMotorCurrent = 0.0;
        public double encoderLeftPosition = 0.0;
        public double encoderRightPosition = 0.0;
        public double endgameAmps = 0.0;
        // current and encoder position variables
    }

    public default void updateInputs(EndgameIOInputs inputs) {}

    public default void setEndgameMotorPower(double leftPercent, double rightPercent) {}
}
