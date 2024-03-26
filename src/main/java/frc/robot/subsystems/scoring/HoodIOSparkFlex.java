package frc.robot.subsystems.scoring;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ScoringConstants;

public class HoodIOSparkFlex implements HoodIO {
    private final CANSparkFlex hoodMotor =
            new CANSparkFlex(ScoringConstants.hoodId, CANSparkFlex.MotorType.kBrushless);

    private final ArmFeedforward feedforward =
            new ArmFeedforward(
                    ScoringConstants.hoodkS, ScoringConstants.hoodkG, ScoringConstants.hoodkV);

    private final TrapezoidProfile profile =
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            ScoringConstants.hoodMaxVelocity,
                            ScoringConstants.hoodMaxAcceleration));

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
        // hoodMotor.setSmartCurrentLimit(150);

        hoodMotor.getPIDController().setP(ScoringConstants.hoodkP);
        hoodMotor.getPIDController().setI(ScoringConstants.hoodkI);
        hoodMotor.getPIDController().setD(ScoringConstants.hoodkD);

        hoodMotor.setInverted(true);

        hoodMotor.getEncoder().setPosition(0.0);
        hoodMotor.getEncoder().setPositionConversionFactor(ScoringConstants.hoodEncoderToRad);
        hoodMotor.getEncoder().setVelocityConversionFactor(ScoringConstants.hoodEncoderToRad);

        hoodMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);

        hoodMotor.setSmartCurrentLimit(140);
    }

    @Override
    public void setHoodAngleRad(double angle) {
        goalAngleRad = angle;

        if (goalAngleRad != previousGoalAngle) {
            profileTimer.reset();
            profileTimer.start();

            initialAngle = hoodMotor.getEncoder().getPosition();
            initialVelocity = hoodMotor.getEncoder().getVelocity();

            previousGoalAngle = goalAngleRad;
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
        // homing = true;
        // hoodMotor.setVoltage(1.5);

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
    public void setBrakeMode(boolean brake) {
        hoodMotor.setIdleMode(brake ? CANSparkFlex.IdleMode.kBrake : CANSparkFlex.IdleMode.kCoast);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        State trapezoidSetpoint =
                profile.calculate(
                        profileTimer.get(),
                        new State(initialAngle, initialVelocity),
                        new State(goalAngleRad, 0.0));

        // if (homing) {
        //     if (hoodMotor.getOutputCurrent() > ScoringConstants.hoodHomeAmps
        //             && homeTimer.get() > 0.4) {
        //         hoodMotor.setVoltage(0);
        //         hoodMotor.getEncoder().setPosition(ScoringConstants.hoodHomeAngleRad);
        //         homing = false;

        //         homeTimer.stop();
        //     }
        // } else if (override) {
        //     hoodMotor.setVoltage(overrideVolts);
        // } else {
        //     hoodMotor
        //             .getPIDController()
        //             .setFF(feedforward.calculate(trapezoidSetpoint.position, 0.0));
        //     hoodMotor
        //             .getPIDController()
        //             .setReference(trapezoidSetpoint.position, ControlType.kPosition);
        // }

        // if (hoodMotor.getOutputCurrent() > 70) {
        //     hoodMotor.setVoltage(0.0);
        // } else {
        hoodMotor.setVoltage(overrideVolts);
        // }

        inputs.hoodAngleRad = hoodMotor.getEncoder().getPosition();
        inputs.hoodGoalAngleRad = trapezoidSetpoint.position;

        inputs.hoodVelocityRadPerSec = hoodMotor.getEncoder().getVelocity();

        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput();
        inputs.hoodStatorCurrentAmps = hoodMotor.getOutputCurrent();
    }
}
