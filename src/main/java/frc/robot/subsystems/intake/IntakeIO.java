package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double leftIntakeVoltage = 0.0;
        public double leftIntakeCurrent = 0.0;

        public double rightIntakeVoltage = 0.0;
        public double rightIntakeCurrent = 0.0;

        public double beltVoltage = 0.0;
        public double beltCurrent = 0.0;

        public boolean noteSensed = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeVoltage(double volts) {}

    public default void setBeltVoltage(double volts) {}
}
