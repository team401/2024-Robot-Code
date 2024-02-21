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
                    2.533,
                    SingleJointedArmSim.estimateMOI(0.1778, 0.68),
                    0.1778,
                    0.0,
                    3.0 * Math.PI / 2.0,
                    false,
                    0.0);
    private final PIDController controller =
            new PIDController(
                    ScoringConstants.hoodkP, ScoringConstants.hoodkI, ScoringConstants.hoodkD);

    private boolean override = false;

    double goalAngleRad = 0.0;
    double appliedVolts = 0.0;

    @Override
    public void setHoodAngleRad(double angle) {
        goalAngleRad = angle;
    }

    @Override
    public void setOverrideMode(boolean override) {
        this.override = override;
    }

    @Override
    public void setOverrideVolts(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void setPID(double p, double i, double d) {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sim.update(Constants.loopTime);

        if (!override) {
            appliedVolts =
                    controller.calculate(sim.getAngleRads(), goalAngleRad)
                            + Math.signum(controller.getPositionError()) * ScoringConstants.hoodkS;
        }

        sim.setInputVoltage(appliedVolts);

        inputs.hoodAngleRad = sim.getAngleRads();
        inputs.hoodGoalAngleRad = goalAngleRad;

        inputs.hoodVelocityRadPerSec = sim.getVelocityRadPerSec();

        inputs.hoodAppliedVolts = appliedVolts;
        inputs.hoodStatorCurrentAmps = sim.getCurrentDrawAmps();
    }
}
