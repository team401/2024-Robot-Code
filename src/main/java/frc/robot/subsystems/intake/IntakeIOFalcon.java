package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SensorConstants;

public class IntakeIOFalcon implements IntakeIO {

    private TalonFX leftIntake = new TalonFX(IntakeConstants.leftIntakeMotorID);

    private TalonFX belt = new TalonFX(IntakeConstants.indexTwoMotorID);

    DigitalInput bannerSensor = new DigitalInput(SensorConstants.uptakeSensorPort);

    public IntakeIOFalcon() {
        leftIntake.setInverted(true);

        belt.setInverted(true);

        TalonFXConfigurator leftConfig = leftIntake.getConfigurator();
        leftConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
        leftConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator beltConfig = belt.getConfigurator();
        beltConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        beltConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftIntakeVoltage = leftIntake.getMotorVoltage().getValueAsDouble();
        inputs.leftIntakeStatorCurrent = leftIntake.getStatorCurrent().getValueAsDouble();

        inputs.beltVoltage = belt.getMotorVoltage().getValueAsDouble();
        inputs.beltStatorCurrent = belt.getStatorCurrent().getValueAsDouble();
        inputs.beltSupplyCurrent = belt.getSupplyCurrent().getValueAsDouble();

        inputs.noteSensed = !bannerSensor.get();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        leftIntake.setControl(new VoltageOut(volts));
    }

    @Override
    public void setBeltVoltage(double volts) {
        belt.setControl(new VoltageOut(volts));
    }
}
