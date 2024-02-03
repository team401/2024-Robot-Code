package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax leftIntake =
            new CANSparkMax(IntakeConstants.leftIntakeMotorID, MotorType.kBrushless);
    private CANSparkMax rightIntake =
            new CANSparkMax(IntakeConstants.rightIntakeMotorID, MotorType.kBrushless);

    private TalonFX frontBelt = new TalonFX(IntakeConstants.frontBeltMotorID);
    private StatusSignal<Double> frontBeltVoltage = frontBelt.getMotorVoltage();
    private StatusSignal<Double> frontBeltCurrent = frontBelt.getStatorCurrent();

    private TalonFX backBelt = new TalonFX(IntakeConstants.backBeltMotorID);
    private StatusSignal<Double> backBeltVoltage = backBelt.getMotorVoltage();
    private StatusSignal<Double> backBeltCurrent = backBelt.getStatorCurrent();

    private DigitalInput bannerSensor = new DigitalInput(IntakeConstants.bannerSensorID);

    public IntakeIOSparkMax() {
        // FIXME: Set up current limits, invert motors as necessary
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.leftIntakeVoltage = leftIntake.getAppliedOutput() * 12;
        inputs.leftIntakeCurrent = leftIntake.getOutputCurrent();

        inputs.rightIntakeVoltage = rightIntake.getAppliedOutput() * 12;
        inputs.rightIntakeCurrent = rightIntake.getOutputCurrent();

        inputs.frontBeltVoltage = frontBeltVoltage.getValueAsDouble();
        inputs.frontBeltCurrent = frontBeltCurrent.getValueAsDouble();

        inputs.backBeltVoltage = backBeltVoltage.getValueAsDouble();
        inputs.backBeltCurrent = backBeltCurrent.getValueAsDouble();

        inputs.noteSensed = bannerSensor.get();
    }

    @Override
    public void setIntakeVoltage(double volts) {
        leftIntake.set(volts / 12);
        rightIntake.set(volts / 12);
    }

    @Override
    public void setBeltVoltage(double volts) {
        frontBelt.setControl(new VoltageOut(volts));
        backBelt.setControl(new VoltageOut(volts));
    }
}
