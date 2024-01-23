package frc.robot.subsystems.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ScoringConstants;

public class HoodIOSim implements HoodIO {
    private final SingleJointedArmSim sim =
            new SingleJointedArmSim(
                    DCMotor.getNeoVortex(1),
                    1.0,
                    SingleJointedArmSim.estimateMOI(0.1, 0.1),
                    0.1,
                    0.0,
                    2.0,
                    false,
                    0.0);
    private final PIDController controller =
            new PIDController(
                    ScoringConstants.aimerkP, ScoringConstants.aimerkI, ScoringConstants.aimerkD);

    double goalAngleRad = 0.0;
    double appliedVolts = 0.0;

    @Override
    public void setHoodAngleRad(double angle) {
        goalAngleRad = angle;
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sim.update(Constants.loopTime);

        appliedVolts = controller.calculate(sim.getAngleRads(), goalAngleRad);
        sim.setInputVoltage(appliedVolts);

        inputs.hoodAngleRad = sim.getAngleRads();
        inputs.hoodGoalAngleRad = goalAngleRad;

        inputs.hoodAppliedVolts = appliedVolts;
        inputs.hoodCurrentAmps = sim.getCurrentDrawAmps();
    }
}
