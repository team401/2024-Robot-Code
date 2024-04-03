package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.endgame.EndgameSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.telemetry.Telemetry;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class EndgameSequence extends Command {
    ScoringSubsystem scoringSubsystem;
    EndgameSubsystem endgameSubsystem;
    CommandSwerveDrivetrain drivetrain;

    Telemetry telemetry;

    BooleanSupplier confirmBooleanSupplier;

    Pose2d targetPose;

    private enum State {
        START,
        CRAWL_UP,
        CLIMB_UP,
        CLIMB_DOWN,
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
        state = State.START;

        // drivetrain.driveToEndgame();
        // drivetrain.setAlignState(AlignState.MANUAL);
        endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_DOWN);
        scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.WAIT);
        scoringSubsystem.forceHood(true);
    }

    @Override
    public void execute() {
        Logger.recordOutput("Endgame/sequenceState", state);
        switch (state) {
            case START:
                if (
                /* drivetrain.atPathfindPose() && */ confirmBooleanSupplier.getAsBoolean()) {
                    state = State.CRAWL_UP;

                    // drivetrain.stopDriveToPose();
                    // drivetrain.setAlignState(AlignState.POSE_TARGET);
                    scoringSubsystem.setAimerStatorCurrentLimit(5.0); // TODO: Find this
                    scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.ENDGAME);
                }
                break;
            case CRAWL_UP:
                // double maxDriveDistance = 0.1;
                // double result =
                //         maxDriveDistance
                //                 * (endgameSubsystem.getPosition()
                //                         - EndgameConstants.climberTargetDownMeters)
                //                 / (EndgameConstants.climberTargetUpMeters
                //                         - EndgameConstants.climberTargetDownMeters);

                // targetPose =
                //         drivetrain
                //                 .getEndgamePose()
                //                 .plus(
                //                         new Transform2d(
                //                                 result
                //                                         * Math.cos(
                //                                                 drivetrain
                //                                                         .getEndgamePose()
                //                                                         .getRotation()
                //                                                         .getRadians()),
                //                                 result
                //                                         * Math.sin(
                //                                                 drivetrain
                //                                                         .getEndgamePose()
                //                                                         .getRotation()
                //                                                         .getRadians()),
                //                                 new Rotation2d()));
                // drivetrain.setPoseTarget(targetPose);

                if (scoringSubsystem.atAimerGoalPosition()
                        && confirmBooleanSupplier.getAsBoolean()) {
                    state = State.CLIMB_UP;

                    // drivetrain.setAlignState(AlignState.MANUAL);
                    // drivetrain.stopDriveToPose();
                    scoringSubsystem.setAimerStatorCurrentLimit(60.0);
                    endgameSubsystem.setClimbing(false);
                    endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_UP);
                }
                break;
            case CLIMB_UP:
                if (endgameSubsystem.atPosition() && confirmBooleanSupplier.getAsBoolean()) {
                    state = State.CLIMB_DOWN;

                    endgameSubsystem.setClimbing(true);
                    endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_DOWN);
                }
                break;
            case CLIMB_DOWN:
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
        // drivetrain.stopDriveToPose();
        // drivetrain.setAlignState(AlignState.MANUAL);
        scoringSubsystem.setAimerStatorCurrentLimit(60.0);
        scoringSubsystem.forceHood(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
