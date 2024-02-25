package frc.robot.subsystems.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
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

    private final TrapezoidProfile profile =
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            ScoringConstants.hoodMaxVelocity,
                            ScoringConstants.hoodMaxAcceleration));

    private boolean override = false;

    private Timer profileTimer = new Timer();

    double initialAngle = 0.0;
    double initialVelocity = 0.0;

    double goalAngleRad = 0.0;
    double previousGoalAngle = 0.0;

    double appliedVolts = 0.0;

    @Override
    public void setHoodAngleRad(double angle) {
        goalAngleRad = angle;

        if (goalAngleRad != previousGoalAngle) {
            profileTimer.reset();
            profileTimer.start();

            initialAngle = sim.getAngleRads();
            initialVelocity = sim.getVelocityRadPerSec();

            previousGoalAngle = goalAngleRad;
        }
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

        State trapezoidSetpoint =
                profile.calculate(
                        profileTimer.get(),
                        new State(initialAngle, initialVelocity),
                        new State(goalAngleRad, 0.0));

        if (!override) {
            appliedVolts =
                    controller.calculate(sim.getAngleRads(), trapezoidSetpoint.position)
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
