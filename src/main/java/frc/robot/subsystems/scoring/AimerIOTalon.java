package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.Constants;
import frc.robot.Constants.ScoringConstants;

public class AimerIOTalon implements AimerIO {
    // TODO: Tune this later
    private final TalonFX armLeft = new TalonFX(ScoringConstants.aimLeftMotorId);
    private final TalonFX armRight = new TalonFX(ScoringConstants.aimRightMotorId);

    private final MotionMagicVoltage controller = new MotionMagicVoltage(0);
    private final ArmFeedforward feedforward = new ArmFeedforward(ScoringConstants.aimerkS, ScoringConstants.aimerkG, ScoringConstants.aimerkV, ScoringConstants.aimerkA);

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ScoringConstants.aimEncoderPort);

    double goalAngleRad = 0.0;

    public AimerIOTalon() {
        armRight.setControl(new Follower(ScoringConstants.aimLeftMotorId, true));

        armLeft.setNeutralMode(NeutralModeValue.Brake);
        armRight.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setAimAngleRad(double angle) {
        goalAngleRad = angle;
    }

    @Override
    public void updateInputs(AimerIOInputs inputs) {
        controller.FeedForward = feedforward.calculate(goalAngleRad, armLeft.getVelocity().getValueAsDouble());

        armLeft.setControl(controller.withPosition(goalAngleRad));

        inputs.aimGoalAngleRad = goalAngleRad;
        inputs.aimAngleRad = encoder.getAbsolutePosition() * 2 * Math.PI;

        inputs.aimAppliedVolts = armLeft.getMotorVoltage().getValueAsDouble();
        inputs.aimCurrentAmps = armLeft.getSupplyCurrent().getValueAsDouble();
    }
}
