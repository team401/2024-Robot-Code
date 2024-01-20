package frc.robot.subsystems.endgame;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface EndgameIO{

    @AutoLog
    public static class EndgameIOInputs{
        public double endgameLeftMotorCurrent = 0.0;
        public double endgameRightMotorCurrent = 0.0;
        public double encoderLeftPosition = 0.0;
        public double encoderRightPosition = 0.0;
        //current and encoder position variables
    }

    public default void updateInputs (EndgameIOInputs inputs){}

    public default void setEndgameMotorPower(double leftPercent, double rightPercent){}

    public default void getEndgamePosition(double position){}

    public default void checkEndgameAmps(double amps){}
}
