package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndgameConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.endgame.EndgameSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.telemetry.Telemetry;
import java.util.function.BooleanSupplier;

public class EndgameSequence extends Command {
    ScoringSubsystem scoringSubsystem;
    EndgameSubsystem endgameSubsystem;
    CommandSwerveDrivetrain drivetrain;

    Telemetry telemetry;

    BooleanSupplier confirmBooleanSupplier;

    Pose2d targetPose;

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
            Telemetry telemetry,
            BooleanSupplier confirmBooleanSupplier) {
        this.scoringSubsystem = scoringSubsystem;
        this.endgameSubsystem = endgameSubsystem;
        this.drivetrain = drivetrain;

        this.telemetry = telemetry;

        this.confirmBooleanSupplier = confirmBooleanSupplier;

        addRequirements(scoringSubsystem);
        addRequirements(endgameSubsystem);
        // addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        state = State.DRIVE_TO_START;

        // drivetrain.driveToEndgame();
        endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_DOWN);
        scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.WAIT);
    }

    @Override
    public void execute() {
        switch (state) {
            case DRIVE_TO_START:
                if (
                /* drivetrain.atPathfindPose() && */ confirmBooleanSupplier.getAsBoolean()) {
                    state = State.MOVE_UP;

                    // drivetrain.stopDriveToPose();
                    endgameSubsystem.setClimbing(false);
                    endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_UP);
                }
                break;
            case MOVE_UP:
                double result =
                        0.1
                                * (endgameSubsystem.getPosition()
                                        - EndgameConstants.climberTargetDownMeters)
                                / (EndgameConstants.climberTargetUpMeters
                                        - EndgameConstants.climberTargetDownMeters);

                targetPose =
                        new Pose2d(
                                drivetrain.getEndgamePose().getX()
                                        + result * Math.cos(telemetry.getRotationRadians()),
                                drivetrain.getEndgamePose().getY()
                                        + result * Math.sin(telemetry.getRotationRadians()),
                                new Rotation2d(telemetry.getRotationRadians()));
                // drivetrain.driveToPose(targetPose);

                if (endgameSubsystem.atPosition() && confirmBooleanSupplier.getAsBoolean()) {
                    state = State.CLIMB;

                    // drivetrain.stopDriveToPose();
                    endgameSubsystem.setClimbing(true);
                    endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_DOWN);
                    scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.ENDGAME);
                }
                break;
            case CLIMB:
                if (endgameSubsystem.atPosition() && confirmBooleanSupplier.getAsBoolean()) {
                    state = State.TRAP;

                    scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.TRAP_SCORE);
                }
                break;
            case TRAP:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stopDriveToPose();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
