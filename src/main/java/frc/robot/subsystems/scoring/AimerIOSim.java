package frc.robot.subsystems.scoring;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.ScoringConstants;

public class AimerIOSim implements AimerIO {
    // TODO: Tune this later
    private final SingleJointedArmSim sim =
            new SingleJointedArmSim(
                    DCMotor.getNeoVortex(2),
                    1.5,
                    SingleJointedArmSim.estimateMOI(0.5, 4.5),
                    0.5,
                    0.0,
                    ScoringConstants.aimMaxAngleRadians,
                    false,
                    0.0);
    private final PIDController controller =
            new PIDController(
                    ScoringConstants.aimerkP, ScoringConstants.aimerkI, ScoringConstants.aimerkD);
    private final ArmFeedforward feedforward =
            new ArmFeedforward(
                    ScoringConstants.aimerkS,
                    ScoringConstants.aimerkG,
                    ScoringConstants.aimerkV,
                    ScoringConstants.aimerkA);
    private final TrapezoidProfile profile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(0.8, 0.5));

    private final Timer timer = new Timer();

    double goalAngleRad = 0.0;
    double appliedVolts = 0.0;

    double initialAngle = 0.0;
    double initialVelocity = 0.0;

    @Override
    public void setAimAngleRad(double angle, boolean newProfile) {
        if (goalAngleRad != angle && newProfile) {
            timer.reset();
            timer.start();

            initialAngle = sim.getAngleRads();
            initialVelocity = sim.getVelocityRadPerSec();
        }
        goalAngleRad = angle;
    }

    @Override
    public void updateInputs(AimerIOInputs inputs) {
        sim.update(Constants.loopTime);

        State trapezoidSetpoint =
                profile.calculate(
                        timer.get(),
                        new State(initialAngle, initialVelocity),
                        new State(goalAngleRad, 0));

        appliedVolts =
                feedforward.calculate(trapezoidSetpoint.position, trapezoidSetpoint.velocity)
                        + controller.calculate(sim.getAngleRads(), trapezoidSetpoint.position);
        sim.setInputVoltage(appliedVolts);

        inputs.aimGoalAngleRad = goalAngleRad;
        inputs.aimAngleRad = sim.getAngleRads();

        inputs.aimAppliedVolts = appliedVolts;
        inputs.aimCurrentAmps = sim.getCurrentDrawAmps();
    }
}
