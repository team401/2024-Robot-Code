package frc.robot.subsystems.endgame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.EndgameConstants;

public class EndgameIOSim implements EndgameIO {
    private ElevatorSim elevatorSim =
            new ElevatorSim(DCMotor.getNeoVortex(2), 20, 1.814, 0.02231009, 0.0, 0.45, true, 0.0);

    private TrapezoidProfile profile =
            new TrapezoidProfile(EndgameConstants.climberProfileConstraints);

    private Timer profileTimer = new Timer();

    boolean override = false;
    double overrideVolts = 0.0;
    double targetPosition = 0.0;

    double initialPosition = 0.0;
    double initialVelocity = 0.0;

    double ff = EndgameConstants.climberkFFClimber;

    private PIDController climberController =
            new PIDController(
                    EndgameConstants.climberkP,
                    EndgameConstants.climberkI,
                    EndgameConstants.climberkD);

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

            initialPosition = elevatorSim.getPositionMeters();
            initialVelocity = elevatorSim.getVelocityMetersPerSecond();

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
    public void setPositionTuning(double position) {
        if (targetPosition != position) {
            profileTimer.reset();
            profileTimer.start();

            targetPosition = position;

            initialPosition = elevatorSim.getPositionMeters();
            initialVelocity = elevatorSim.getVelocityMetersPerSecond();
        }
    }

    @Override
    public void updateInputs(EndgameIOInputs inputs) {
        elevatorSim.update(Constants.loopTime);

        double appliedVolts = 0.0;

        if (override) {
            appliedVolts = overrideVolts;
        } else {
            State trapezoidSetpoint =
                    profile.calculate(
                            profileTimer.get(),
                            new State(initialPosition, initialVelocity),
                            new State(targetPosition, 0.0));
            inputs.targetPosition = trapezoidSetpoint.position;
            appliedVolts =
                    ff
                            + climberController.calculate(
                                    elevatorSim.getPositionMeters(), trapezoidSetpoint.position);
        }

        elevatorSim.setInputVoltage(appliedVolts);
        inputs.endgameLeftAppliedVolts = appliedVolts;
        inputs.endgameLeftStatorCurrentAmps = elevatorSim.getCurrentDrawAmps();

        inputs.endgameRightAppliedVolts = appliedVolts;
        inputs.endgameRightStatorCurrentAmps = elevatorSim.getCurrentDrawAmps();

        inputs.position = elevatorSim.getPositionMeters();
        inputs.velocity = elevatorSim.getVelocityMetersPerSecond();
    }

    @Override
    public void setPID(double p, double i, double d) {
        climberController.setP(p);
        climberController.setI(i);
        climberController.setD(d);
    }

    @Override
    public void setFF(double ff) {
        this.ff = ff;
    }

    @Override
    public void setMaxProfile(double maxVelocity, double maxAcceleration) {
        profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
    }
}
