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

    private TalonFX belt = new TalonFX(IntakeConstants.frontBeltMotorID);
    private StatusSignal<Double> beltVoltage = belt.getMotorVoltage();
    private StatusSignal<Double> beltCurrent = belt.getStatorCurrent();

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

        inputs.beltVoltage = beltVoltage.getValueAsDouble();
        inputs.beltCurrent = beltCurrent.getValueAsDouble();

        inputs.noteSensed = bannerSensor.get();
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
