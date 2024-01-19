package frc.robot.subsystems.scoring;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ScoringConstants;

public class AimerIOSim implements AimerIO {
    // TODO: Tune this later
    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNeoVortex(2), 1.0, 1.0, 1.0, 0.0, 1.0, true, 0.0);
    private final PIDController controller = new PIDController(ScoringConstants.aimerkP, ScoringConstants.aimerkI, ScoringConstants.aimerkD);
    private final ArmFeedforward feedforward = new ArmFeedforward(ScoringConstants.aimerkS, ScoringConstants.aimerkG, ScoringConstants.aimerkV, ScoringConstants.aimerkA);

    double goalAngRad = 0.0;
    double appliedVolts = 0.0;

    @Override
    public void setAimAngleRad(double angle) {
        goalAngRad = angle;
    }

    @Override
    public void updateInputs(AimerIOInputs inputs) {
        sim.update(Constants.loopTime);

        appliedVolts =
                feedforward.calculate(sim.getAngleRads(), sim.getVelocityRadPerSec())
                        + controller.calculate(sim.getAngleRads(), goalAngRad);
        sim.setInputVoltage(appliedVolts);

        inputs.aimGoalAngleRad = goalAngRad;
        inputs.aimAngleRad = sim.getAngleRads();

        inputs.aimAppliedVolts = appliedVolts;
        inputs.aimCurrentAmps = sim.getCurrentDrawAmps();
    }
}
