package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double frontMotorVoltage = 0.0;
        public double frontMotorCurrent = 0.0;

        public double backMotorVoltage = 0.0;
        public double backMotorCurrent = 0.0;

        public double beltMotorVoltage = 0.0;
        public double beltMotorCurrent = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeVoltage(double volts) {}

    public default void setBeltVoltage(double volts) {}
}
