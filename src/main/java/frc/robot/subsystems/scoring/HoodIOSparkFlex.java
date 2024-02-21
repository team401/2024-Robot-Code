package frc.robot.subsystems.scoring;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ScoringConstants;

public class HoodIOSparkFlex implements HoodIO {
    private final CANSparkFlex hoodMotor =
            new CANSparkFlex(ScoringConstants.hoodId, CANSparkFlex.MotorType.kBrushless);

    private final ArmFeedforward feedforward =
            new ArmFeedforward(
                    ScoringConstants.hoodkS, ScoringConstants.hoodkG, ScoringConstants.hoodkV);

    private final TrapezoidProfile profile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(0.5, 0.5));

    private boolean override = false;
    private boolean homing = false;

    double overrideVolts = 0.0;

    double goalAngleRad = 0.0;
    double previousGoalAngle = 0.1;

    double initialAngle = 0.0;
    double initialVelocity = 0.0;

    private Timer profileTimer = new Timer();
    private Timer homeTimer = new Timer();

    public HoodIOSparkFlex() {
        hoodMotor.setSmartCurrentLimit(120);

        hoodMotor.getPIDController().setP(ScoringConstants.hoodkP);
        hoodMotor.getPIDController().setI(ScoringConstants.hoodkI);
        hoodMotor.getPIDController().setD(ScoringConstants.hoodkD);

        hoodMotor.setInverted(true);

        hoodMotor.getEncoder().setPosition(0.0);
        hoodMotor.getEncoder().setPositionConversionFactor(ScoringConstants.hoodEncoderToRad);
        hoodMotor.getEncoder().setVelocityConversionFactor(ScoringConstants.hoodEncoderToRad);

        hoodMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
    }

    @Override
    public void setHoodAngleRad(double angle) {
        goalAngleRad = angle;

        if (goalAngleRad != previousGoalAngle) {
            profileTimer.reset();
            profileTimer.start();
            // X
        }
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
        homing = true;
        hoodMotor.setVoltage(1.5);

        homeTimer.reset();
        homeTimer.start();
    }

    @Override
    public void setPID(double p, double i, double d) {
        hoodMotor.getPIDController().setP(p);
        hoodMotor.getPIDController().setI(i);
        hoodMotor.getPIDController().setD(d);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // State trapezoidSetpoint = new State(
        //     profile.calculate(profileTimer.get(), new State(), )
        // );
        if (homing) {
            if (hoodMotor.getOutputCurrent() > ScoringConstants.hoodHomeAmps
                    && homeTimer.get() > 0.4) {
                hoodMotor.setVoltage(0);
                hoodMotor.getEncoder().setPosition(ScoringConstants.hoodHomeAngleRad);
                homing = false;

                homeTimer.stop();
            }
        } else if (override) {
            hoodMotor.setVoltage(overrideVolts);
        } else {
            hoodMotor.getPIDController().setFF(feedforward.calculate(goalAngleRad, 0.0));
            hoodMotor.getPIDController().setReference(goalAngleRad, ControlType.kPosition);
        }

        inputs.hoodAngleRad = hoodMotor.getEncoder().getPosition();
        inputs.hoodGoalAngleRad = goalAngleRad;

        inputs.hoodVelocityRadPerSec = hoodMotor.getEncoder().getVelocity();

        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput();
        inputs.hoodStatorCurrentAmps = hoodMotor.getOutputCurrent();
    }
}
