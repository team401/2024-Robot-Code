package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.SensorConstants;

public class ShooterIOTalon implements ShooterIO {
    private final TalonFX kicker = new TalonFX(ScoringConstants.kickerMotorId);

    private final TalonFX shooterLeft = new TalonFX(ScoringConstants.shooterLeftMotorId);
    private final TalonFX shooterRight = new TalonFX(ScoringConstants.shooterRightMotorId);

    MotionMagicVelocityVoltage leftController = new MotionMagicVelocityVoltage(0).withSlot(0);
    MotionMagicVelocityVoltage rightController = new MotionMagicVelocityVoltage(0).withSlot(0);

    private final MotionMagicConfigs configs = new MotionMagicConfigs();
    private final Slot0Configs slot0 = new Slot0Configs();

    DigitalInput bannerSensor = new DigitalInput(SensorConstants.upperBannerPort);

    private boolean override = false;
    private double overrideVolts = 0.0;

    double goalLeftVelocityRPM = 0.0;
    double goalRightVelocityRPM = 0.0;

    public ShooterIOTalon() {
        kicker.setInverted(true);

        shooterLeft.setInverted(true);
        shooterRight.setInverted(false);

        shooterLeft.setNeutralMode(NeutralModeValue.Coast);
        shooterRight.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfigurator shooterLeftConfig = shooterLeft.getConfigurator();
        shooterLeftConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true));

        TalonFXConfigurator shooterRightConfig = shooterRight.getConfigurator();
        shooterRightConfig.apply(
                new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(120)
                        .withStatorCurrentLimitEnable(true));

        slot0.withKP(ScoringConstants.shooterkP);
        slot0.withKI(ScoringConstants.shooterkI);
        slot0.withKD(ScoringConstants.shooterkD);

        slot0.withKS(ScoringConstants.shooterkS);
        slot0.withKV(ScoringConstants.shooterkV);
        slot0.withKA(ScoringConstants.shooterkA);

        configs.withMotionMagicAcceleration(ScoringConstants.shooterAcceleration);
        configs.withMotionMagicJerk(ScoringConstants.shooterJerk);

        shooterLeft.getConfigurator().apply(slot0);
        shooterRight.getConfigurator().apply(slot0);

        shooterLeft.getConfigurator().apply(configs);
        shooterRight.getConfigurator().apply(configs);
    }

    @Override
    public void setShooterVelocityRPM(double velocity) {
        goalLeftVelocityRPM = velocity;
        goalRightVelocityRPM = velocity * ScoringConstants.shooterOffsetAdjustment;
    }

    @Override
    public void setKickerVolts(double volts) {
        kicker.setVoltage(volts);
    }

    @Override
    public void setOverrideMode(boolean override) {
        this.override = override;
    }

    @Override
    public void setOverrideVolts(double volts) {
        overrideVolts = volts;
    }

    @Override
    public void setPID(double p, double i, double d) {
        slot0.withKP(p);
        slot0.withKI(i);
        slot0.withKD(d);

        shooterLeft.getConfigurator().apply(slot0);
        shooterRight.getConfigurator().apply(slot0);
    }

    @Override
    public void setMaxAcceleration(double maxAcceleration) {
        configs.withMotionMagicAcceleration(maxAcceleration);

        shooterLeft.getConfigurator().apply(configs);
        shooterRight.getConfigurator().apply(configs);
    }

    @Override
    public void setMaxJerk(double maxJerk) {
        configs.withMotionMagicJerk(maxJerk);

        shooterLeft.getConfigurator().apply(configs);
        shooterRight.getConfigurator().apply(configs);
    }

    @Override
    public void setFF(double kS, double kV, double kA) {
        slot0.withKS(kS);
        slot0.withKV(kV);
        slot0.withKA(kA);

        shooterLeft.getConfigurator().apply(slot0);
        shooterRight.getConfigurator().apply(slot0);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        if (override) {
            shooterLeft.setVoltage(overrideVolts);
            shooterRight.setVoltage(overrideVolts);
        } else {
            shooterLeft.setControl(
                    leftController.withVelocity(
                            goalLeftVelocityRPM / ConversionConstants.kMinutesToSeconds));
            shooterRight.setControl(
                    rightController.withVelocity(
                            goalRightVelocityRPM / ConversionConstants.kMinutesToSeconds));
        }

        inputs.shooterLeftVelocityRPM =
                shooterLeft.getVelocity().getValueAsDouble()
                        / ConversionConstants.kSecondsToMinutes;
        inputs.shooterLeftGoalVelocityRPM = goalLeftVelocityRPM;
        inputs.shooterLeftAppliedVolts = shooterLeft.getMotorVoltage().getValueAsDouble();
        inputs.shooterLeftCurrentAmps = shooterLeft.getStatorCurrent().getValueAsDouble();
        inputs.shooterLeftSupplyCurrentAmps = shooterLeft.getSupplyCurrent().getValueAsDouble();

        inputs.shooterRightVelocityRPM =
                shooterRight.getVelocity().getValueAsDouble()
                        / ConversionConstants.kSecondsToMinutes;
        inputs.shooterRightGoalVelocityRPM = goalRightVelocityRPM;
        inputs.shooterRightAppliedVolts = shooterRight.getMotorVoltage().getValueAsDouble();
        inputs.shooterRightCurrentAmps = shooterRight.getStatorCurrent().getValueAsDouble();
        inputs.shooterRightSupplyCurrentAmps = shooterRight.getSupplyCurrent().getValueAsDouble();

        inputs.kickerAppliedVolts = kicker.getMotorVoltage().getValueAsDouble();
        inputs.kickerCurrentAmps = kicker.getStatorCurrent().getValueAsDouble();

        inputs.bannerSensor = bannerSensor.get();
    }
}
