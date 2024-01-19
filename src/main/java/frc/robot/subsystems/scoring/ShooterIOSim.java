package frc.robot.subsystems.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.Scoring;

public class ShooterIOSim implements ShooterIO {
    // TODO: Tune this later
    private final FlywheelSim shooterSim = new FlywheelSim(DCMotor.getNeoVortex(1), 1.0, 1.0);
    private final PIDController shooterController =
            new PIDController(Scoring.shooterkP, Scoring.shooterkI, Scoring.shooterkD);
    private final SimpleMotorFeedforward shooterFeedforward =
            new SimpleMotorFeedforward(Scoring.shooterkS, Scoring.shooterkV, Scoring.shooterkA);

    double shooterAppliedVolts = 0.0;
    double shooterGoalVelRPM = 0.0;
    double kickerVolts = 0.0;

    @Override
    public void setShooterVelocityRPM(double vel) {
        shooterGoalVelRPM = vel;
    }

    @Override
    public void setKickerVolts(double volts) {
        kickerVolts = volts;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterSim.update(Constants.loopTime);

        shooterAppliedVolts =
                shooterFeedforward.calculate(shooterSim.getAngularVelocityRadPerSec())
                        + shooterController.calculate(
                                shooterSim.getAngularVelocityRadPerSec(),
                                shooterGoalVelRPM * 2.0 * Math.PI / 60.0);

        inputs.shooterVelocityRPM =
                shooterSim.getAngularVelocityRadPerSec() * 60.0 / (2.0 * Math.PI);
        inputs.shooterGoalVelocityRPM = shooterGoalVelRPM;
        inputs.shooterAppliedVolts = shooterAppliedVolts;
        inputs.shooterCurrentAmps = shooterSim.getCurrentDrawAmps();

        inputs.kickerAppliedVolts = kickerVolts;
        inputs.kickerCurrentAmps = 0.0;
    }
}
