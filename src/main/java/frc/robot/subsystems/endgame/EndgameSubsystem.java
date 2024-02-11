package frc.robot.subsystems.endgame;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndgameSubsystem extends SubsystemBase {
    private final EndgameIO endgameIo;
    private final EndgameIOInputsAutoLogged endgameInputs = new EndgameIOInputsAutoLogged();

    public enum EndgameAction {
        GO_UP,
        GO_DOWN,
        CANCEL
    }

    public EndgameSubsystem(EndgameIO endgameIO) {
        this.endgameIo = endgameIO;
    }

    public void setAction(EndgameAction action) {
        switch (action) {
            case GO_UP:
                endgameIo.setVolts(4.0);
                Logger.recordOutput("endgame/State", action);
                break;
            case GO_DOWN:
                endgameIo.setVolts(-4.0);
                Logger.recordOutput("endgame/State", action);
                break;
            case CANCEL:
                endgameIo.setVolts(0.0);
                Logger.recordOutput("endgame/State", action);
                break;
        }
    }

    public double getPosition() {
        return endgameInputs.position;
    }

    public void home() {}

    @Override
    public void periodic() {
        endgameIo.updateInputs(endgameInputs);
        Logger.processInputs("endgame", endgameInputs);

        Logger.recordOutput(
                "endgame/Elevator3d",
                new Pose3d(0.0, 0.0, endgameInputs.position + 0.1, new Rotation3d(0, 0, 0)));
    }
}
