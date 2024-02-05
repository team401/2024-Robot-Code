package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.ScoringConstants;

public class ShooterIOTalon implements ShooterIO {
    private final TalonFX kicker = new TalonFX(ScoringConstants.kickerMotorId);

    private final TalonFX shooterLeft = new TalonFX(ScoringConstants.shooterLeftMotorId);
    private final TalonFX shooterRight = new TalonFX(ScoringConstants.shooterRightMotorId);

    MotionMagicVelocityVoltage leftController = new MotionMagicVelocityVoltage(0).withSlot(0);
    MotionMagicVelocityVoltage rightController = new MotionMagicVelocityVoltage(0).withSlot(0);

    private final MotionMagicConfigs configs = new MotionMagicConfigs();
    private final Slot0Configs slot0 = new Slot0Configs();

    DigitalInput bannerSensor = new DigitalInput(Constants.SensorConstants.bannerPort);

    double goalLeftVelocityRPM = 0.0;
    double goalRightVelocityRPM = 0.0;

    public ShooterIOTalon() {
        shooterRight.setInverted(true);

        shooterLeft.setNeutralMode(NeutralModeValue.Coast);
        shooterRight.setNeutralMode(NeutralModeValue.Coast);

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
    public void updateInputs(ShooterIOInputs inputs) {
        shooterLeft.setControl(leftController.withVelocity(goalLeftVelocityRPM));
        shooterRight.setControl(rightController.withVelocity(goalRightVelocityRPM));

        inputs.shooterLeftVelocityRPM =
                shooterLeft.getVelocity().getValueAsDouble()
                        * ConversionConstants.kSecondsToMinutes;
        inputs.shooterLeftGoalVelocityRPM = goalLeftVelocityRPM;
        inputs.shooterLeftAppliedVolts = shooterLeft.getMotorVoltage().getValueAsDouble();
        inputs.shooterLeftCurrentAmps = shooterLeft.getSupplyCurrent().getValueAsDouble();

        inputs.shooterRightVelocityRPM =
                shooterRight.getVelocity().getValueAsDouble()
                        * ConversionConstants.kSecondsToMinutes;
        inputs.shooterRightGoalVelocityRPM = goalRightVelocityRPM;
        inputs.shooterRightAppliedVolts = shooterRight.getMotorVoltage().getValueAsDouble();
        inputs.shooterRightCurrentAmps = shooterRight.getSupplyCurrent().getValueAsDouble();

        inputs.kickerAppliedVolts = kicker.getMotorVoltage().getValueAsDouble();
        inputs.kickerCurrentAmps = kicker.getSupplyCurrent().getValueAsDouble();

        inputs.bannerSensor = bannerSensor.get();
    }
}
