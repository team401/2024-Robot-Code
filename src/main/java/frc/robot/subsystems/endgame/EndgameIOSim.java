package frc.robot.subsystems.endgame;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class EndgameIOSim implements EndgameIO {
    private final ElevatorSim elevatorSim =
            new ElevatorSim(DCMotor.getNeoVortex(2), 0.05, 0.1, 0.05, 0.0, 0.45, false, 0.0);

    double appliedVolts = 0.0;

    @Override
    public void setVolts(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void updateInputs(EndgameIOInputs inputs) {
        elevatorSim.update(Constants.loopTime);
        elevatorSim.setInputVoltage(appliedVolts);

        inputs.endgameLeftAppliedVolts = appliedVolts;
        inputs.endgameLeftCurrentAmps = elevatorSim.getCurrentDrawAmps();

        inputs.endgameRightAppliedVolts = appliedVolts;
        inputs.endgameRightCurrentAmps = elevatorSim.getCurrentDrawAmps();

        inputs.position = elevatorSim.getPositionMeters();
    }
}
