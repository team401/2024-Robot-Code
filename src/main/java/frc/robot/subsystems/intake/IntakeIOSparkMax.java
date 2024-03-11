package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SensorConstants;

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax leftIntake =
            new CANSparkMax(IntakeConstants.leftIntakeMotorID, MotorType.kBrushless);
    private CANSparkMax rightIntake =
            new CANSparkMax(IntakeConstants.rightIntakeMotorID, MotorType.kBrushless);

    private TimeOfFlight uptakeSensor = new TimeOfFlight(SensorConstants.uptakeSensorPort);

    private TalonFX belt = new TalonFX(IntakeConstants.indexTwoMotorID);

    public IntakeIOSparkMax() {
        leftIntake.setSmartCurrentLimit(40, 40);
        rightIntake.setSmartCurrentLimit(40, 40);

        leftIntake.setInverted(true);
        rightIntake.setInverted(true);

        belt.setInverted(true);

        TalonFXConfigurator beltConfig = belt.getConfigurator();
        beltConfig.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        beltConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(60)
                        .withStatorCurrentLimitEnable(true));

        uptakeSensor.setRangeOfInterest(
                SensorConstants.uptakeTopLeftX,
                SensorConstants.uptakeTopLeftY,
                SensorConstants.uptakeBottomRightX,
                SensorConstants.uptakeBottomRightY);
        uptakeSensor.setRangingMode(RangingMode.Short, 40.0); // runs every other loop
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftIntakeVoltage = leftIntake.getAppliedOutput() * 12;
        inputs.leftIntakeStatorCurrent = leftIntake.getOutputCurrent();

        inputs.rightIntakeVoltage = rightIntake.getAppliedOutput() * 12;
        inputs.rightIntakeStatorCurrent = rightIntake.getOutputCurrent();

        inputs.beltVoltage = belt.getMotorVoltage().getValueAsDouble();
        inputs.beltStatorCurrent = belt.getStatorCurrent().getValueAsDouble();
        inputs.beltSupplyCurrent = belt.getSupplyCurrent().getValueAsDouble();

        if (uptakeSensor.isRangeValid()) {
            inputs.noteSensed = uptakeSensor.getRange() < SensorConstants.maxRange;
        } else {
            inputs.noteSensed = false; // no distance was measured
        }
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
