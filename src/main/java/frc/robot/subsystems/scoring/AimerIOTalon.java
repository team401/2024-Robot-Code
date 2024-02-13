package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ScoringConstants;

public class AimerIOTalon implements AimerIO {
    private final TalonFX aimerLeft = new TalonFX(ScoringConstants.aimLeftMotorId);
    private final TalonFX aimerRight = new TalonFX(ScoringConstants.aimRightMotorId);

    private final MotionMagicVoltage controller = new MotionMagicVoltage(0).withSlot(0);
    private final MotionMagicConfigs configs = new MotionMagicConfigs();
    private final Slot0Configs slot0 = new Slot0Configs();

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ScoringConstants.aimEncoderPort);

    private boolean override = false;
    private double overrideVolts = 0.0;

    double goalAngleRad = 0.0;

    double minAngleClamp = 0.0;
    double maxAngleClamp = 0.0;

    double lastPosition = 0.0;
    double lastTime = Utils.getCurrentTimeSeconds();

    public AimerIOTalon() {
        aimerRight.setControl(new Follower(ScoringConstants.aimLeftMotorId, true));

        aimerLeft.setNeutralMode(NeutralModeValue.Brake);
        aimerRight.setNeutralMode(NeutralModeValue.Brake);

        slot0.withKP(ScoringConstants.aimerkP);
        slot0.withKI(ScoringConstants.aimerkI);
        slot0.withKD(ScoringConstants.aimerkD);

        slot0.withKS(ScoringConstants.aimerkS);
        slot0.withKG(ScoringConstants.aimerkG);
        slot0.withKV(ScoringConstants.aimerkV);
        slot0.withKA(ScoringConstants.aimerkA);

        configs.withMotionMagicAcceleration(ScoringConstants.aimAcceleration);
        configs.withMotionMagicCruiseVelocity(ScoringConstants.aimCruiseVelocity);

        aimerLeft.getConfigurator().apply(slot0);
        aimerRight.getConfigurator().apply(slot0);

        aimerLeft.getConfigurator().apply(configs);
        aimerRight.getConfigurator().apply(configs);

        encoder.setDistancePerRotation(2 * Math.PI);
    }

    @Override
    public void setAimAngleRad(double goalAngleRad, boolean newProfile) {
        this.goalAngleRad = goalAngleRad;
    }

    @Override
    public void controlAimAngleRad() {
        goalAngleRad = MathUtil.clamp(goalAngleRad, minAngleClamp, maxAngleClamp);
    }

    @Override
    public void setAngleClampsRad(double minAngleClamp, double maxAngleClamp) {
        this.minAngleClamp = minAngleClamp;
        this.maxAngleClamp = maxAngleClamp;
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
    public void updateInputs(AimerIOInputs inputs) {
        if (override) {
            aimerLeft.setVoltage(overrideVolts);
        } else {
            aimerLeft.setControl(controller.withPosition(goalAngleRad));
        }

        inputs.aimGoalAngleRad = goalAngleRad;
        inputs.aimAngleRad = encoder.getAbsolutePosition();

        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;

        inputs.aimVelocityRadPerSec = (encoder.getAbsolutePosition() - lastPosition) / diffTime;
        lastPosition = encoder.getAbsolutePosition();

        inputs.aimAppliedVolts = aimerLeft.getMotorVoltage().getValueAsDouble();
        inputs.aimCurrentAmps = aimerLeft.getSupplyCurrent().getValueAsDouble();
    }
}
