package frc.robot.subsystems.endgame;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.EndgameConstants;

public class EndgameIOSparkFlex implements EndgameIO {
    private final CANSparkFlex leftEndgameMotor =
            new CANSparkFlex(EndgameConstants.leftMotorID, MotorType.kBrushless);
    private final CANSparkFlex rightEndgameMotor =
            new CANSparkFlex(EndgameConstants.rightMotorID, MotorType.kBrushless);

    boolean override = false;

    public EndgameIOSparkFlex() {
        leftEndgameMotor.setSmartCurrentLimit(EndgameConstants.smartCurrentLimit);
        rightEndgameMotor.setSmartCurrentLimit(EndgameConstants.smartCurrentLimit);

        rightEndgameMotor.setIdleMode(IdleMode.kBrake);
        leftEndgameMotor.setIdleMode(IdleMode.kBrake);

        leftEndgameMotor.follow(rightEndgameMotor, true);

        leftEndgameMotor.getEncoder().setPositionConversionFactor(EndgameConstants.encoderToMeters);
        leftEndgameMotor.getEncoder().setPosition(0.0);

        leftEndgameMotor.getPIDController().setP(EndgameConstants.climberkP);
        leftEndgameMotor.getPIDController().setI(EndgameConstants.climberkI);
        leftEndgameMotor.getPIDController().setD(EndgameConstants.climberkD);

        rightEndgameMotor.getPIDController().setP(EndgameConstants.climberkP);
        rightEndgameMotor.getPIDController().setI(EndgameConstants.climberkI);
        rightEndgameMotor.getPIDController().setD(EndgameConstants.climberkD);
    }

    @Override
    public void setOverrideMode(boolean override) {
        this.override = override;
    }

    @Override
    public void setOverrideVolts(double volts) {
        rightEndgameMotor.setVoltage(volts);
    }

    @Override
    public void setPosition(double position) {
        rightEndgameMotor.getPIDController().setReference(position, ControlType.kPosition);
    }

    @Override
    public void updateInputs(EndgameIOInputs inputs) {
        inputs.endgameLeftAppliedVolts = leftEndgameMotor.getAppliedOutput();
        inputs.endgameLeftStatorCurrentAmps = leftEndgameMotor.getOutputCurrent();

        inputs.endgameRightAppliedVolts = rightEndgameMotor.getAppliedOutput();
        inputs.endgameRightStatorCurrentAmps = rightEndgameMotor.getOutputCurrent();

        inputs.position = leftEndgameMotor.getEncoder().getPosition();
        inputs.velocity = leftEndgameMotor.getEncoder().getVelocity();
    }

    @Override
    public void setPID(double p, double i, double d) {
        leftEndgameMotor.getPIDController().setP(p);
        leftEndgameMotor.getPIDController().setI(i);
        leftEndgameMotor.getPIDController().setD(d);

        rightEndgameMotor.getPIDController().setP(p);
        rightEndgameMotor.getPIDController().setI(i);
        rightEndgameMotor.getPIDController().setD(d);
    }
}
