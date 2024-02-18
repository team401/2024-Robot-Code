package frc.robot.subsystems.scoring;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ScoringConstants;

public class HoodIOSparkFlex implements HoodIO {
    private final CANSparkFlex hoodMotor =
            new CANSparkFlex(ScoringConstants.hoodId, CANSparkFlex.MotorType.kBrushless);

    private boolean override = false;
    private boolean homing = false;

    double overrideVolts = 0.0;

    double goalAngleRad = 0.0;

    private Timer homeTimer = new Timer();

    public HoodIOSparkFlex() {
        hoodMotor.setSmartCurrentLimit(120);

        hoodMotor.getPIDController().setP(ScoringConstants.hoodkP);
        hoodMotor.getPIDController().setI(ScoringConstants.hoodkI);
        hoodMotor.getPIDController().setD(ScoringConstants.hoodkD);
        hoodMotor.getPIDController().setFF(ScoringConstants.hoodkFF);

        hoodMotor.setInverted(true);

        hoodMotor.getEncoder().setPosition(0.0);
        hoodMotor.getEncoder().setPositionConversionFactor(ScoringConstants.hoodEncoderToRad);
        hoodMotor.getEncoder().setVelocityConversionFactor(ScoringConstants.hoodEncoderToRad);

        hoodMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
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
    public void home() {
        // homing = true;
        // hoodMotor.setVoltage(0.5);

        // homeTimer.reset();
        // homeTimer.start();
    }

    @Override
    public void setPID(double p, double i, double d) {
        hoodMotor.getPIDController().setP(p);
        hoodMotor.getPIDController().setI(i);
        hoodMotor.getPIDController().setD(d);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        if (homing) {
            if (hoodMotor.getOutputCurrent() > ScoringConstants.hoodHomeAmps
                    && homeTimer.get() > 1.0) {
                // hoodMotor.setVoltage(0);
                hoodMotor.getEncoder().setPosition(ScoringConstants.hoodHomeAngleRad);
                homing = false;

                homeTimer.stop();
            }
        } else if (override) {
            // hoodMotor.setVoltage(overrideVolts);
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
