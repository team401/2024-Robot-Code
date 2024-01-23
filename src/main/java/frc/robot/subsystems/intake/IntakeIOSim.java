package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSim implements IntakeIO {
    /*
     * Simulating the wheels directly doesn't make much sense here, so we can
     * instead use SmartDashboard to puppeteer a note through the indexer
     */

    // FIXME: find a more reasonable way to pantomime the intake. Maybe a timer?

    // 10 and 11 aren't valid DIO channels on the real robot
    private DigitalInput noteInIntakeWheels = new DigitalInput(10);
    private DigitalInput noteInBelts = new DigitalInput(11);

    private double intakeWheelsAppliedVolts = 0.0;
    private double beltAppliedVolts = 0.0;

    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.backMotorVoltage = intakeWheelsAppliedVolts;
        inputs.backMotorCurrent = noteInIntakeWheels.get() ? 100000 : 0;

        inputs.frontMotorVoltage = intakeWheelsAppliedVolts;
        inputs.frontMotorCurrent = noteInIntakeWheels.get() ? 100000 : 0;

        inputs.beltMotorVoltage = beltAppliedVolts;
        inputs.beltMotorCurrent = noteInBelts.get() ? 100000 : 0;
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
