package frc.robot.subsystems.endgame;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.EndgameConstants;

public class EndgameIOSparkFlex implements EndgameIO {
    private final CANSparkFlex leftEndgameMotor =
            new CANSparkFlex(EndgameConstants.leftMotorID, MotorType.kBrushless);
    private final CANSparkFlex rightEndgameMotor =
            new CANSparkFlex(EndgameConstants.rightMotorID, MotorType.kBrushless);

    private TrapezoidProfile profile =
            new TrapezoidProfile(EndgameConstants.climberProfileConstraints);

    private Timer profileTimer = new Timer();

    boolean override = false;
    double overrideVolts = 0.0;
    double targetPosition = 0.0;

    double initialPosition = 0.0;
    double initialVelocity = 0.0;

    public EndgameIOSparkFlex() {
        leftEndgameMotor.setSmartCurrentLimit(EndgameConstants.smartCurrentLimit);
        rightEndgameMotor.setSmartCurrentLimit(EndgameConstants.smartCurrentLimit);

        rightEndgameMotor.setIdleMode(IdleMode.kBrake);
        leftEndgameMotor.setIdleMode(IdleMode.kBrake);

        leftEndgameMotor.follow(rightEndgameMotor, true);

        leftEndgameMotor.getEncoder().setPositionConversionFactor(EndgameConstants.encoderToMeters);
        leftEndgameMotor.getEncoder().setPosition(0.0);

        rightEndgameMotor.getPIDController().setP(EndgameConstants.climberkP);
        rightEndgameMotor.getPIDController().setI(EndgameConstants.climberkI);
        rightEndgameMotor.getPIDController().setD(EndgameConstants.climberkD);
        rightEndgameMotor.getPIDController().setFF(EndgameConstants.climberkFFClimber);
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
    public void setPosition(double position) {
        if (targetPosition != position) {
            profileTimer.reset();
            profileTimer.start();

            targetPosition = position;

            initialPosition = rightEndgameMotor.getEncoder().getPosition();
            initialVelocity = rightEndgameMotor.getEncoder().getVelocity();

            if (position < targetPosition) {
                // Moving down, assume weight of robot is on the climber now
                setFF(EndgameConstants.climberkFFRobot);
            } else {
                // Moving up, we only have to account for the weight of pushing the climber up
                setFF(EndgameConstants.climberkFFClimber);
            }
        }
    }

    @Override
    public void setBrakeMode(boolean brake) {
        IdleMode sparkMode = brake ? IdleMode.kBrake : IdleMode.kCoast;
        leftEndgameMotor.setIdleMode(sparkMode);
    }

    public void setPositionTuning(double position) {
        if (targetPosition != position) {
            profileTimer.reset();
            profileTimer.start();

            targetPosition = position;

            initialPosition = rightEndgameMotor.getEncoder().getPosition();
            initialVelocity = rightEndgameMotor.getEncoder().getVelocity();
        }
    }

    @Override
    public void updateInputs(EndgameIOInputs inputs) {
        if (override) {
            rightEndgameMotor.setVoltage(overrideVolts);
        } else {
            State trapezoidSetpoint =
                    profile.calculate(
                            profileTimer.get(),
                            new State(initialPosition, initialVelocity),
                            new State(targetPosition, 0.0));
            double clampedPosition =
                    MathUtil.clamp(
                            trapezoidSetpoint.position,
                            EndgameConstants.climberMinPositionMeters,
                            EndgameConstants.climberMaxPositionMeters);
            rightEndgameMotor
                    .getPIDController()
                    .setReference(clampedPosition, ControlType.kPosition);
        }

        inputs.endgameLeftAppliedVolts = leftEndgameMotor.getAppliedOutput();
        inputs.endgameLeftStatorCurrentAmps = leftEndgameMotor.getOutputCurrent();

        inputs.endgameRightAppliedVolts = rightEndgameMotor.getAppliedOutput();
        inputs.endgameRightStatorCurrentAmps = rightEndgameMotor.getOutputCurrent();

        inputs.position = leftEndgameMotor.getEncoder().getPosition();
        inputs.velocity = leftEndgameMotor.getEncoder().getVelocity();
    }

    @Override
    public void setPID(double p, double i, double d) {
        rightEndgameMotor.getPIDController().setP(p);
        rightEndgameMotor.getPIDController().setI(i);
        rightEndgameMotor.getPIDController().setD(d);
    }

    @Override
    public void setFF(double ff) {
        rightEndgameMotor.getPIDController().setFF(ff);
    }

    @Override
    public void setMaxProfile(double maxVelocity, double maxAcceleration) {
        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }
}
