package frc.robot.subsystems.endgame;

import edu.wpi.first.math.VecBuilder;
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

    private final TrapezoidProfile profile =
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

    // Were we moving down on last tick?
    // This is used to keep track of whether or not we need to switch the elevatorSim object out.
    boolean movingDownLast = false;

    @Override
    public void updateInputs(EndgameIOInputs inputs) {
        elevatorSim.update(Constants.loopTime);

        if (targetPosition < elevatorSim.getPositionMeters() && !movingDownLast) {
            // We were not moving down before, but now we are, so we must update the sim to be
            // simulating lifting the whole weight of the robot up
            ElevatorSim newElevatorSim =
                    new ElevatorSim(
                            DCMotor.getNeoVortex(2),
                            20,
                            1.814
                                    - EndgameConstants
                                            .simRobotMass, // Add in negative robot mass, because
                            // the
                            // robot will be pulling up on the elevator.
                            0.02231009,
                            0.0,
                            0.45,
                            true,
                            0.0);
            newElevatorSim.setState(
                    VecBuilder.fill(
                            elevatorSim.getPositionMeters(),
                            elevatorSim.getVelocityMetersPerSecond()));
            elevatorSim = newElevatorSim;

            // Update state of direction we're currently moving
            movingDownLast = true;
        } else if (movingDownLast) {
            // We were moving down before, but now we are not. Create a new simulator without the
            // weight from lifting the robot.
            ElevatorSim newElevatorSim =
                    new ElevatorSim(
                            DCMotor.getNeoVortex(2), 20, 1.814, 0.02231009, 0.0, 0.45, true, 0.0);
            newElevatorSim.setState(
                    VecBuilder.fill(
                            elevatorSim.getPositionMeters(),
                            elevatorSim.getVelocityMetersPerSecond()));
            elevatorSim = newElevatorSim;

            // Update state of direction we're currently moving
            movingDownLast = false;
        }

        double appliedVolts = 0.0;

        if (override) {
            appliedVolts = overrideVolts;
        } else {
            State trapezoidSetpoint =
                    profile.calculate(
                            profileTimer.get(),
                            new State(initialPosition, initialVelocity),
                            new State(targetPosition, 0.0));
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
}
