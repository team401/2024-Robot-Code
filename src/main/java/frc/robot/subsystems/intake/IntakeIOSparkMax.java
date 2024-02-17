package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax leftIntake =
            new CANSparkMax(IntakeConstants.leftIntakeMotorID, MotorType.kBrushless);
    private CANSparkMax rightIntake =
            new CANSparkMax(IntakeConstants.rightIntakeMotorID, MotorType.kBrushless);

    private TalonFX belt = new TalonFX(IntakeConstants.indexTwoMotorID);
    private StatusSignal<Double> beltVoltage = belt.getMotorVoltage();
    private StatusSignal<Double> beltCurrent = belt.getStatorCurrent();

    // private DigitalInput bannerSensor = new DigitalInput(IntakeConstants.bannerSensorID);

    public IntakeIOSparkMax() {
        leftIntake.setSmartCurrentLimit(20, 25);
        rightIntake.setSmartCurrentLimit(20, 25);

        TalonFXConfigurator beltConfig = belt.getConfigurator();
        beltConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        beltConfig.apply(
                new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(30)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentThreshold(35)
                        .withSupplyTimeThreshold(1));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftIntakeVoltage = leftIntake.getAppliedOutput() * 12;
        inputs.leftIntakeCurrent = leftIntake.getOutputCurrent();

        inputs.rightIntakeVoltage = rightIntake.getAppliedOutput() * 12;
        inputs.rightIntakeCurrent = rightIntake.getOutputCurrent();

        inputs.beltVoltage = beltVoltage.getValueAsDouble();
        inputs.beltCurrent = beltCurrent.getValueAsDouble();

        // inputs.noteSensed = bannerSensor.get();
        inputs.noteSensed = false;
    }

    @Override
    public void setIntakeVoltage(double volts) {
        leftIntake.set(volts / 12);
        rightIntake.set(volts / 12);
    }

    @Override
    public void setBeltVoltage(double volts) {
        belt.setControl(new VoltageOut(volts));
    }
}
