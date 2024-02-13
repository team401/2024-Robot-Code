package frc.robot.subsystems.scoring;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import frc.robot.Constants.ScoringConstants;

public class HoodIOSparkFlex implements HoodIO {
    private final CANSparkFlex hoodMotor =
            new CANSparkFlex(ScoringConstants.hoodId, CANSparkFlex.MotorType.kBrushless);

    private boolean override = false;

    double overrideVolts = 0.0;

    double goalAngleRad = 0.0;

    public HoodIOSparkFlex() {
        hoodMotor.setSmartCurrentLimit(10);

        hoodMotor.getPIDController().setP(ScoringConstants.hoodkP);
        hoodMotor.getPIDController().setI(ScoringConstants.hoodkI);
        hoodMotor.getPIDController().setD(ScoringConstants.hoodkD);
        hoodMotor.getPIDController().setFF(ScoringConstants.hoodkFF);

        hoodMotor.getEncoder().setPosition(0.0);
        hoodMotor.getEncoder().setPositionConversionFactor(ScoringConstants.hoodEncoderToRad);

        hoodMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        hoodMotor.getEncoder().setPositionConversionFactor(15 / 38 * 2 * Math.PI);
        hoodMotor.getEncoder().setVelocityConversionFactor(15 / 38 * 2 * Math.PI);
    }

    @Override
    public void setHoodAngleRad(double angle) {
        goalAngleRad = angle;
    }

    @Override
    public void setOverrideMode(boolean override) {
        this.override = override;
    }

    @Override
    public void setOverrideVolts(double volts) {
        overrideVolts = volts;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        if (override) {
            hoodMotor.setVoltage(overrideVolts);
        } else {
            hoodMotor.getPIDController().setReference(goalAngleRad, ControlType.kPosition);
        }

        inputs.hoodAngleRad = hoodMotor.getEncoder().getPosition();
        inputs.hoodGoalAngleRad = goalAngleRad;

        inputs.hoodVelocityRadPerSec = hoodMotor.getEncoder().getVelocity();

        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput();
        inputs.hoodCurrentAmps = hoodMotor.getOutputCurrent();
    }
}
