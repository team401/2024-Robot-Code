package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {
        public double leftIntakeVoltage = 0.0;
        public double leftIntakeStatorCurrent = 0.0;
        public double leftIntakeSupplyCurrent = 0.0;

        public double rightIntakeVoltage = 0.0;
        public double rightIntakeStatorCurrent = 0.0;
        public double rightIntakeSupplyCurrent = 0.0;

        public double beltVoltage = 0.0;
        public double beltStatorCurrent = 0.0;
        public double beltSupplyCurrent = 0.0;

        public boolean noteSensed = false;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeVoltage(double volts) {}

    public default void setBeltVoltage(double volts) {}

    public default boolean checkIfStopIntake() {
        return false;
    }
}
