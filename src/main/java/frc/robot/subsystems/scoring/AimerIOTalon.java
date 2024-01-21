package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ScoringConstants;

public class AimerIOTalon implements AimerIO {
    // TODO: Tune this later
    private final TalonFX armLeft = new TalonFX(ScoringConstants.aimLeftMotorId);
    private final TalonFX armRight = new TalonFX(ScoringConstants.aimRightMotorId);

    private final MotionMagicVoltage controller = new MotionMagicVoltage(0).withSlot(0);
    private final ArmFeedforward feedforward =
            new ArmFeedforward(
                    ScoringConstants.aimerkS,
                    ScoringConstants.aimerkG,
                    ScoringConstants.aimerkV,
                    ScoringConstants.aimerkA);
    private final Slot0Configs slot0 = new Slot0Configs();

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ScoringConstants.aimEncoderPort);

    double goalAngleRad = 0.0;

    public AimerIOTalon() {
        armRight.setControl(new Follower(ScoringConstants.aimLeftMotorId, true));

        armLeft.setNeutralMode(NeutralModeValue.Brake);
        armRight.setNeutralMode(NeutralModeValue.Brake);

        slot0.withKP(ScoringConstants.aimerkP);
        slot0.withKI(ScoringConstants.aimerkI);
        slot0.withKD(ScoringConstants.aimerkD);

        slot0.withKS(ScoringConstants.aimerkS);
        slot0.withKG(ScoringConstants.aimerkG);
        slot0.withKV(ScoringConstants.aimerkV);
        slot0.withKA(ScoringConstants.aimerkA);

        armLeft.getConfigurator().apply(slot0);
        armRight.getConfigurator().apply(slot0);
    }

    @Override
    public void setAimAngleRad(double angle) {
        goalAngleRad = angle;
    }

    @Override
    public void updateInputs(AimerIOInputs inputs) {
        controller.withFeedForward(
                feedforward.calculate(goalAngleRad, armLeft.getVelocity().getValueAsDouble()));

        armLeft.setControl(controller.withPosition(goalAngleRad).withSlot(0));

        inputs.aimGoalAngleRad = goalAngleRad;
        inputs.aimAngleRad = encoder.getAbsolutePosition() * 2 * Math.PI;

        inputs.aimAppliedVolts = armLeft.getMotorVoltage().getValueAsDouble();
        inputs.aimCurrentAmps = armLeft.getSupplyCurrent().getValueAsDouble();
    }
}
