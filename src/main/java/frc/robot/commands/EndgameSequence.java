package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.endgame.EndgameSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import java.util.function.BooleanSupplier;

public class EndgameSequence extends Command {
    ScoringSubsystem scoringSubsystem;
    EndgameSubsystem endgameSubsystem;
    CommandSwerveDrivetrain drivetrain;

    BooleanSupplier confirmBooleanSupplier;

    private enum State {
        DRIVE_TO_START,
        MOVE_UP,
        CLIMB,
        TRAP
    };

    private State state;

    public EndgameSequence(
            ScoringSubsystem scoringSubsystem,
            EndgameSubsystem endgameSubsystem,
            CommandSwerveDrivetrain drivetrain,
            BooleanSupplier confirmBooleanSupplier) {
        this.scoringSubsystem = scoringSubsystem;
        this.endgameSubsystem = endgameSubsystem;
        this.drivetrain = drivetrain;

        this.confirmBooleanSupplier = confirmBooleanSupplier;

        addRequirements(scoringSubsystem);
        addRequirements(endgameSubsystem);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        state = State.DRIVE_TO_START;

        drivetrain.driveToEndgame();
        endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_DOWN);
        scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.WAIT);
    }

    @Override
    public void execute() {
        switch (state) {
            case DRIVE_TO_START:
                if (drivetrain.atPathfindPose()) {
                    state = State.MOVE_UP;

                    drivetrain.stopDriveToPose();
                    drivetrain.setGoalChassisSpeeds(new ChassisSpeeds(0.1, 0.0, 0.0), false);
                    endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_UP);
                }
                break;
            case MOVE_UP:
                if (confirmBooleanSupplier.getAsBoolean()) {
                    state = State.CLIMB;

                    endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_DOWN);
                    scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.ENDGAME);
                }
                break;
            case CLIMB:
                if (confirmBooleanSupplier.getAsBoolean()) {
                    state = State.TRAP;
                }
                break;
            case TRAP:
                endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.TEMPORARY_SETPOINT);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
