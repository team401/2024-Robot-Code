package frc.robot.subsystems.scoring;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.Scoring;

public class AimerIOSim implements AimerIO {
    // TODO: Tune this later
    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNeoVortex(2), 1.0, 0.0, 1.0, 0.0, 1.0, true, 0.0);
    private final PIDController controller = new PIDController(Scoring.aimerkP, Scoring.aimerkI, Scoring.aimerkD);
    private final ArmFeedforward feedforward = new ArmFeedforward(Scoring.aimerkS, Scoring.aimerkG, Scoring.aimerkV, Scoring.aimerkA);

    double goalAngRad = 0.0;
    double appliedVolts = 0.0;

    @Override
    public void setAimAngleRad(double angle) {
        goalAngRad = angle;
    }

    @Override
    public void updateInputs(AimerIOInputs inputs) {
        sim.update(Constants.loopTime);

        appliedVolts = feedforward.calculate(sim.getAngleRads(), sim.getVelocityRadPerSec()) + controller.calculate(sim.getAngleRads(), goalAngRad);
        sim.setInputVoltage(appliedVolts);

        inputs.aimAngleRad = sim.getAngleRads();

        inputs.aimAppliedVolts = appliedVolts;
        inputs.aimCurrentAmps = sim.getCurrentDrawAmps();
    }
}
