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

    private double setpointPosition = 0.0;

    private boolean climbing = false;

    public enum EndgameAction {
        GO_UP,
        GO_DOWN,
        WAIT,
        OVERRIDE,
        TEMPORARY_SETPOINT
    }

    private enum State {
        OVERRIDE, // Targetting a voltage
        GO_UP,
        GO_DOWN,
        CANCEL,
        TEMPORARY_SETPOINT
    }

    private State state = State.CANCEL;
    private EndgameAction action = EndgameAction.WAIT;

    public EndgameSubsystem(EndgameIO endgameIO) {
        this.endgameIo = endgameIO;
    }

    public void setAction(EndgameAction action) {
        this.action = action;

        switch (action) {
            case GO_UP:
                state = State.GO_UP;
                break;
            case GO_DOWN:
                state = State.GO_DOWN;
                break;
            case WAIT:
                state = State.CANCEL;
                break;
            case OVERRIDE:
                state = State.OVERRIDE;
                break;
            case TEMPORARY_SETPOINT:
                state = State.TEMPORARY_SETPOINT;
                break;
        }
    }

    public void flipDirection() {
        setAction(
                setpointPosition == EndgameConstants.climberTargetDownMeters
                        ? EndgameAction.GO_UP
                        : EndgameAction.GO_DOWN);
    }

    public boolean atPosition() {
        return Math.abs(setpointPosition - endgameInputs.position) < 0.07;
    }

    public void setBrakeMode(boolean brake) {
        endgameIo.setBrakeMode(brake);
    }

    public double getPosition() {
        return endgameInputs.position;
    }

    public void setClimbing(boolean climbing) {
        endgameIo.setClimbing(climbing);
    }

    @Override
    public double getPosition(int slot) {
        return getPosition();
    }

    public void home() {}

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
        endgameIo.setPID(p, i, d);
    }

    @Override
    public void runToPosition(double position, int slot) {
        endgameIo.setPositionTuning(position);
    }

    @Override
    public void periodic() {
        endgameIo.updateInputs(endgameInputs);
        Logger.processInputs("endgame", endgameInputs);

        switch (state) {
            case GO_UP:
                setpointPosition = EndgameConstants.climberTargetUpMeters;
                endgameIo.setPosition(setpointPosition);
                endgameIo.setOverrideMode(false);
                break;
            case GO_DOWN:
                setpointPosition = EndgameConstants.climberTargetDownMeters;
                endgameIo.setPosition(setpointPosition);
                endgameIo.setOverrideMode(false);
                break;
            case CANCEL:
                endgameIo.setOverrideVolts(0.0);
                endgameIo.setOverrideMode(true);
                break;
            case OVERRIDE:
                endgameIo.setOverrideMode(true);
                break;
            case TEMPORARY_SETPOINT:
                endgameIo.setOverrideMode(false);
                break;
        }

        Logger.recordOutput("endgame/Action", action);

        Logger.recordOutput("endgame/State", state);

        if (state == State.OVERRIDE) {
            endgameIo.setOverrideMode(true);
            if (Math.abs(endgameInputs.position) > 0.52 && overrideVolts > 0) {
                endgameIo.setOverrideVolts(0.0);
            } else {
                endgameIo.setOverrideVolts(overrideVolts);
            }
        } else {
            endgameIo.setOverrideMode(false);
        }

        Logger.recordOutput(
                "endgame/Elevator3d",
                new Pose3d(0.0, 0.0, endgameInputs.position + 0.1, new Rotation3d(0, 0, 0)));
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG, int slot) {
        endgameIo.setFF(kG);
    }

    @Override
    public void setMaxProfileProperties(double maxVelocity, double maxAcceleration, int slot) {
        endgameIo.setMaxProfile(maxVelocity, maxAcceleration);
    }
}
