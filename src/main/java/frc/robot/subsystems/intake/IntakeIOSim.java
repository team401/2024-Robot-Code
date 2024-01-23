package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOSim implements IntakeIO {
    /*
     * Simulating the wheels directly doesn't make much sense here, so we can
     * instead use SmartDashboard to puppeteer a note through the indexer
     */
    // FIXME: find a more reasonable way to pantomime the intake. Maybe a timer?
    private boolean noteInIntakeWheels = false;
    private boolean noteInBelts = false;

    private double intakeWheelsAppliedVolts = 0.0;
    private double beltAppliedVolts = 0.0;

    public IntakeIOSim() {
        SmartDashboard.putBoolean("noteInIntake", false);
        SmartDashboard.putBoolean("noteInBelts", false);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        noteInIntakeWheels = SmartDashboard.getBoolean("noteInIntake", false);
        noteInBelts = SmartDashboard.getBoolean("noteInBelts", false);

        inputs.backMotorVoltage = intakeWheelsAppliedVolts;
        inputs.backMotorCurrent = noteInIntakeWheels ? 100000 : 0;

        inputs.frontMotorVoltage = intakeWheelsAppliedVolts;
        inputs.frontMotorCurrent = noteInIntakeWheels ? 100000 : 0;

        inputs.beltMotorVoltage = beltAppliedVolts;
        inputs.beltMotorCurrent = noteInBelts ? 100000 : 0;
    }

    @Override
    public void setIntakeVoltage(double volts) {
        intakeWheelsAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setBeltVoltage(double volts) {
        beltAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
