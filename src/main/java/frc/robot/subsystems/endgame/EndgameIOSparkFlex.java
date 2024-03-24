package frc.robot.subsystems.endgame;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.REVLibError;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.EndgameConstants;
import org.littletonrobotics.junction.Logger;

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
    double goalPosition = 0.0;

    double profileSetpoint = 0.0;

    double initialPosition = 0.0;
    double initialVelocity = 0.0;

    public EndgameIOSparkFlex() {

        rightEndgameMotor.setIdleMode(IdleMode.kBrake);
        leftEndgameMotor.setIdleMode(IdleMode.kBrake);

        rightEndgameMotor.setInverted(false);
        leftEndgameMotor.follow(rightEndgameMotor, true);

        leftEndgameMotor.getEncoder().setPositionConversionFactor(EndgameConstants.encoderToMeters);
        leftEndgameMotor.getEncoder().setPosition(0.0);

        rightEndgameMotor
                .getEncoder()
                .setPositionConversionFactor(EndgameConstants.encoderToMeters);
        rightEndgameMotor.getEncoder().setPosition(0.0);

        rightEndgameMotor.getPIDController().setP(EndgameConstants.climberkP);
        rightEndgameMotor.getPIDController().setI(EndgameConstants.climberkI);
        rightEndgameMotor.getPIDController().setD(EndgameConstants.climberkD);
        rightEndgameMotor.getPIDController().setFF(EndgameConstants.climberkFFClimber);

        rightEndgameMotor.getPIDController().setFeedbackDevice(rightEndgameMotor.getEncoder());
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
    public void setClimbing(boolean climbing) {
        rightEndgameMotor
                .getPIDController()
                .setFF(
                        climbing
                                ? EndgameConstants.climberkFFRobot
                                : EndgameConstants.climberkFFClimber);
    }

    @Override
    public void setPosition(double position) {
        if (goalPosition != position) {
            profileTimer.reset();
            profileTimer.start();

            initialPosition = rightEndgameMotor.getEncoder().getPosition();
            initialVelocity = rightEndgameMotor.getEncoder().getVelocity();

            if (position < goalPosition) {
                // Moving down, assume weight of robot is on the climber now
                setFF(EndgameConstants.climberkFFRobot);
            } else {
                // Moving up, we only have to account for the weight of pushing the climber up
                setFF(EndgameConstants.climberkFFClimber);
            }
        }
        goalPosition = position;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        IdleMode sparkMode = brake ? IdleMode.kBrake : IdleMode.kCoast;
        leftEndgameMotor.setIdleMode(sparkMode);
        rightEndgameMotor.setIdleMode(sparkMode);
    }

    public void setPositionTuning(double position) {
        if (goalPosition != position) {
            profileTimer.reset();
            profileTimer.start();

            initialPosition = rightEndgameMotor.getEncoder().getPosition();
            initialVelocity = rightEndgameMotor.getEncoder().getVelocity();
        }
        goalPosition = position;
    }

    @Override
    public void updateInputs(EndgameIOInputs inputs) {
        REVLibError err;
        if (override) {
            SmartDashboard.putNumber("endgame/overrideVolts", overrideVolts);
            rightEndgameMotor.setVoltage(overrideVolts);

            err = REVLibError.kOk;
        } else {
            State trapezoidSetpoint =
                    profile.calculate(
                            profileTimer.get(),
                            // HACK: our velocity calculation is too noisy to be used, so we assume
                            // it's zero
                            new State(initialPosition, 0),
                            new State(goalPosition, 0.0));
            Logger.recordOutput("endgame/initialPosition", initialPosition);
            Logger.recordOutput("endgame/initialVelocity", initialVelocity);
            Logger.recordOutput("endgame/trapezoidSetpoint", trapezoidSetpoint.position);

            double clampedPosition =
                    MathUtil.clamp(
                            trapezoidSetpoint.position,
                            EndgameConstants.climberMinPositionMeters,
                            EndgameConstants.climberMaxPositionMeters);

            profileSetpoint = clampedPosition;

            err =
                    rightEndgameMotor
                            .getPIDController()
                            .setReference(clampedPosition, ControlType.kPosition);
        }

        Logger.recordOutput("endgame/overrideMode", override);
        Logger.recordOutput("endgame/motorOk", err == REVLibError.kOk);

        inputs.endgameLeftAppliedVolts = leftEndgameMotor.getAppliedOutput();
        inputs.endgameLeftStatorCurrentAmps = leftEndgameMotor.getOutputCurrent();

        inputs.endgameRightAppliedVolts = rightEndgameMotor.getAppliedOutput();
        inputs.endgameRightStatorCurrentAmps = rightEndgameMotor.getOutputCurrent();

        inputs.overrideVolts = this.overrideVolts;

        inputs.finalTargetPosition = goalPosition;
        inputs.profileTargetPosition = profileSetpoint;

        inputs.position = rightEndgameMotor.getEncoder().getPosition();
        inputs.velocity = leftEndgameMotor.getEncoder().getVelocity();

        inputs.profileTimerTime = profileTimer.get();
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
