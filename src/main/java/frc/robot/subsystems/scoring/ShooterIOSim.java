package frc.robot.subsystems.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.ScoringConstants;

public class ShooterIOSim implements ShooterIO {
    // TODO: Tune this later
    private final FlywheelSim shooterLeftSim =
            new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.0010639061);
    private final FlywheelSim shooterRightSim =
            new FlywheelSim(DCMotor.getKrakenX60(1), 0.5, 0.0010639061);

    private final PIDController shooterLeftController =
            new PIDController(
                    ScoringConstants.shooterkP,
                    ScoringConstants.shooterkI,
                    ScoringConstants.shooterkD);
    private final PIDController shooterRightController =
            new PIDController(
                    ScoringConstants.shooterkP,
                    ScoringConstants.shooterkI,
                    ScoringConstants.shooterkD);

    private final SimpleMotorFeedforward shooterFeedforward =
            new SimpleMotorFeedforward(
                    ScoringConstants.shooterkS,
                    ScoringConstants.shooterkV,
                    ScoringConstants.shooterkA);

    private boolean override = false;

    DigitalInput bannerSensor = new DigitalInput(Constants.SensorConstants.bannerPort);

    double shooterLeftGoalVelRPM = 0.0;
    double shooterLeftAppliedVolts = 0.0;

    double shooterRightGoalVelRPM = 0.0;
    double shooterRightAppliedVolts = 0.0;

    double kickerVolts = 0.0;

    @Override
    public void setShooterVelocityRPM(double velocity) {
        shooterLeftGoalVelRPM = velocity;
        shooterRightGoalVelRPM = velocity * ScoringConstants.shooterOffsetAdjustment;
    }

    @Override
    public void setKickerVolts(double volts) {
        kickerVolts = volts;
    }

    @Override
    public void setOverrideMode(boolean override) {
        this.override = override;
    }

    @Override
    public void setOverrideVolts(double volts) {
        shooterLeftAppliedVolts = volts;
        shooterRightAppliedVolts = volts;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        shooterLeftSim.update(Constants.loopTime);
        shooterRightSim.update(Constants.loopTime);

        if (!override) {
            shooterLeftAppliedVolts =
                    shooterFeedforward.calculate(shooterLeftSim.getAngularVelocityRadPerSec())
                            + shooterLeftController.calculate(
                                    shooterLeftSim.getAngularVelocityRadPerSec(),
                                    shooterLeftGoalVelRPM
                                            * ConversionConstants.kRPMToRadiansPerSecond);
            shooterRightAppliedVolts =
                    shooterFeedforward.calculate(shooterRightSim.getAngularVelocityRadPerSec())
                            + shooterRightController.calculate(
                                    shooterRightSim.getAngularVelocityRadPerSec(),
                                    shooterRightGoalVelRPM
                                            * ConversionConstants.kRPMToRadiansPerSecond);
        }

        shooterLeftSim.setInputVoltage(shooterLeftAppliedVolts);
        shooterRightSim.setInputVoltage(shooterRightAppliedVolts);

        inputs.shooterLeftVelocityRPM =
                shooterLeftSim.getAngularVelocityRadPerSec()
                        * ConversionConstants.kRadiansPerSecondToRPM;
        inputs.shooterLeftGoalVelocityRPM = shooterLeftGoalVelRPM;
        inputs.shooterLeftAppliedVolts = shooterLeftAppliedVolts;
        inputs.shooterLeftCurrentAmps = shooterLeftSim.getCurrentDrawAmps();

        inputs.shooterRightVelocityRPM =
                shooterRightSim.getAngularVelocityRadPerSec()
                        * ConversionConstants.kRadiansPerSecondToRPM;
        inputs.shooterRightGoalVelocityRPM = shooterRightGoalVelRPM;
        inputs.shooterRightAppliedVolts = shooterRightAppliedVolts;
        inputs.shooterRightCurrentAmps = shooterRightSim.getCurrentDrawAmps();

        inputs.kickerAppliedVolts = kickerVolts;
        inputs.kickerCurrentAmps = 0.0;

        inputs.bannerSensor = bannerSensor.get();
    }
}
