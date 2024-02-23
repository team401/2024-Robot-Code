package frc.robot.subsystems.endgame;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndgameConstants;
import frc.robot.utils.Tunable;
import org.littletonrobotics.junction.Logger;

public class EndgameSubsystem extends SubsystemBase implements Tunable {
    private final EndgameIO endgameIo;
    private final EndgameIOInputsAutoLogged endgameInputs = new EndgameIOInputsAutoLogged();

    private double overrideVolts = 0.0;

    public enum EndgameAction {
        GO_UP,
        GO_DOWN,
        CANCEL,
        OVERRIDE
    }

    private enum State {
        OVERRIDE,
        NORMAL
    }

    private State state = State.NORMAL;

    public EndgameSubsystem(EndgameIO endgameIO) {
        this.endgameIo = endgameIO;
    }

    public void setAction(EndgameAction action) {
        switch (action) {
            case GO_UP:
                endgameIo.setPosition(EndgameConstants.climberTargetUpMeters);
                endgameIo.setOverrideMode(false);
                state = State.NORMAL;
                break;
            case GO_DOWN:
                endgameIo.setPosition(EndgameConstants.climberTargetDownMeters);
                endgameIo.setOverrideMode(false);
                state = State.NORMAL;
                break;
            case CANCEL:
                endgameIo.setOverrideVolts(0.0);
                endgameIo.setOverrideMode(true);
                state = State.NORMAL;
                break;
            case OVERRIDE:
                endgameIo.setOverrideMode(true);
                state = State.OVERRIDE;
                break;
        }

        Logger.recordOutput("endgame/Action", action);
    }

    public double getPosition() {
        return endgameInputs.position;
    }

    public void home() {}

    @Override
    public double getPosition(int slot) {
        return endgameInputs.position;
    }

    @Override
    public double getVelocity(int slot) {
        return endgameInputs.velocity;
    }

    @Override
    public double getConversionFactor(int slot) {
        return 0.45;
    }

    @Override
    public void setVolts(double volts, int slot) {
        overrideVolts = volts;
    }

    @Override
    public void setPID(double p, double i, double d, int slot) {
        switch (slot) {
            case 0:
                endgameIo.setPID(p, i, d);
            default:
                throw new IllegalArgumentException("Invalid slot");
        }
    }

    @Override
    public void runToPosition(double position, int slot) {
        throw new UnsupportedOperationException("Unimplemented method 'runToPosition'");
    }

    @Override
    public void periodic() {
        endgameIo.updateInputs(endgameInputs);
        Logger.processInputs("endgame", endgameInputs);

        Logger.recordOutput("endgame/State", state);

        if (state == State.OVERRIDE) {
            endgameIo.setOverrideMode(true);
            endgameIo.setOverrideVolts(overrideVolts);
        } else {
            endgameIo.setOverrideMode(false);
        }

        Logger.recordOutput(
                "endgame/Elevator3d",
                new Pose3d(0.0, 0.0, endgameInputs.position + 0.1, new Rotation3d(0, 0, 0)));
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG, int slot) {
        throw new UnsupportedOperationException("Unimplemented method 'setFF'");
    }

    @Override
    public void setMaxProfileProperties(double maxVelocity, double maxAcceleration, int slot) {
        throw new UnsupportedOperationException("Unimplemented method 'setMaxProfileVelocity'");
    }
}
