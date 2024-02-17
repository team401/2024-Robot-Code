package frc.robot.subsystems.endgame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.EndgameConstants;

public class EndgameIOSim implements EndgameIO {
    private final ElevatorSim elevatorSim =
            new ElevatorSim(DCMotor.getNeoVortex(2), 20, 1.814, 0.02231009, 0.0, 0.45, true, 0.0);

    double appliedVolts = 0.0;

    private PIDController climberController = new PIDController(EndgameConstants.climberkP, EndgameConstants.climberkI, EndgameConstants.climberkD);

    @Override
    public void setVolts(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void updateInputs(EndgameIOInputs inputs) {
        elevatorSim.update(Constants.loopTime);
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
}
