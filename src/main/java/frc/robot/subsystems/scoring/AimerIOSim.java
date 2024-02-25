package frc.robot.subsystems.scoring;

import edu.wpi.first.math.MathUtil;
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
    private final SingleJointedArmSim sim =
            new SingleJointedArmSim(
                    DCMotor.getFalcon500(2),
                    80,
                    SingleJointedArmSim.estimateMOI(0.3872, 8.61),
                    0.4,
                    0.0,
                    ScoringConstants.aimMaxAngleRadians + Math.PI / 2.0,
                    true,
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
            new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(
                            ScoringConstants.aimCruiseVelocity, ScoringConstants.aimAcceleration));

    private final Timer timer = new Timer();

    private boolean override = false;
    private double overrideVolts = 0.0;

    boolean newProfile = false;
    double previousGoalAngle = 0.0;

    double minAngleClamp = 0.0;
    double maxAngleClamp = 0.0;

    double goalAngleRad = 0.0;
    double appliedVolts = 0.0;

    double initialAngle = 0.0;
    double initialVelocity = 0.0;

    @Override
    public void setAimAngleRad(double goalAngleRad, boolean newProfile) {
        this.goalAngleRad = goalAngleRad;
        this.newProfile = newProfile;
    }

    @Override
    public void controlAimAngleRad() {
        if (goalAngleRad != previousGoalAngle && newProfile) {
            timer.reset();
            timer.start();

            initialAngle = sim.getAngleRads();
            initialVelocity = sim.getVelocityRadPerSec();

            previousGoalAngle = goalAngleRad;
        }
        goalAngleRad = MathUtil.clamp(goalAngleRad, minAngleClamp, maxAngleClamp);
    }

    @Override
    public void setAngleClampsRad(double minAngleClamp, double maxAngleClamp) {
        if (minAngleClamp > maxAngleClamp) {
            return;
        }
        this.minAngleClamp =
                MathUtil.clamp(minAngleClamp, 0.0, ScoringConstants.aimMaxAngleRadians);
        this.maxAngleClamp =
                MathUtil.clamp(maxAngleClamp, 0.0, ScoringConstants.aimMaxAngleRadians);
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
    public void updateInputs(AimerIOInputs inputs) {
        sim.update(Constants.loopTime);

        State trapezoidSetpoint =
                profile.calculate(
                        timer.get(),
                        new State(initialAngle, initialVelocity),
                        new State(goalAngleRad, 0));

        double controlSetpoint =
                MathUtil.clamp(
                        trapezoidSetpoint.position, 0.0, ScoringConstants.aimMaxAngleRadians);
        double velocitySetpoint = trapezoidSetpoint.velocity;

        if (override) {
            sim.setInputVoltage(overrideVolts);
        } else {
            appliedVolts =
                    feedforward.calculate(controlSetpoint, velocitySetpoint)
                            + controller.calculate(sim.getAngleRads(), controlSetpoint);
            appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
            sim.setInputVoltage(appliedVolts);
        }

        inputs.aimGoalAngleRad = goalAngleRad;
        inputs.aimProfileGoalAngleRad = trapezoidSetpoint.position;
        inputs.aimAngleRad = sim.getAngleRads();

        inputs.aimVelocityRadPerSec = sim.getVelocityRadPerSec();

        inputs.aimAppliedVolts = appliedVolts;
        inputs.aimStatorCurrentAmps = sim.getCurrentDrawAmps();
    }
}
