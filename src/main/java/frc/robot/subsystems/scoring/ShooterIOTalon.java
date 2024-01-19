package frc.robot.subsystems.scoring;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ScoringConstants;

public class ShooterIOTalon implements ShooterIO {
    private final TalonFX kicker = new TalonFX(ScoringConstants.kickerMotorId);

    private final TalonFX shooterLeft = new TalonFX(ScoringConstants.shooterLeftMotorId);
    private final TalonFX shooterRight = new TalonFX(ScoringConstants.shooterRightMotorId);

    MotionMagicVelocityVoltage controller = new MotionMagicVelocityVoltage(0);
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ScoringConstants.shooterkS, ScoringConstants.shooterkV, ScoringConstants.shooterkA);

    double goalVelocityRPM = 0.0;

    public ShooterIOTalon() {
        shooterRight.setControl(new Follower(ScoringConstants.shooterLeftMotorId, true));

        shooterLeft.setNeutralMode(NeutralModeValue.Coast);
        shooterRight.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void setShooterVelocityRPM(double velocity) {
        goalVelocityRPM = velocity;
    }

    @Override
    public void setKickerVolts(double volts) {
        kicker.setVoltage(volts);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        controller.FeedForward = feedforward.calculate(goalVelocityRPM);

        shooterLeft.setControl(controller.withVelocity(goalVelocityRPM));

        inputs.shooterVelocityRPM = shooterLeft.getVelocity().getValueAsDouble();
        inputs.shooterGoalVelocityRPM = goalVelocityRPM;
        inputs.shooterAppliedVolts = shooterLeft.getMotorVoltage().getValueAsDouble();
        inputs.shooterCurrentAmps = shooterLeft.getSupplyCurrent().getValueAsDouble();

        inputs.kickerAppliedVolts = kicker.getMotorVoltage().getValueAsDouble();
        inputs.kickerCurrentAmps = kicker.getSupplyCurrent().getValueAsDouble();
    }
}
