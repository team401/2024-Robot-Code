package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutomatedTeleopConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeAction;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import frc.robot.telemetry.Telemetry;

public class AutomatedTeleop extends Command {
    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;
    CommandSwerveDrivetrain drivetrain;

    Telemetry telemetry;

    private Timer timer;

    private enum State {
        START,
        DRIVE_TO_SOURCE,
        INTAKE_NOTE,
        RETRY_INTAKE,
        DRIVE_TO_SCORING,
    }

    private State state;

    public AutomatedTeleop(
            ScoringSubsystem scoringSubsystem,
            IntakeSubsystem intakeSubsystem,
            CommandSwerveDrivetrain drivetrain,
            Telemetry telemetry) {
        this.scoringSubsystem = scoringSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.drivetrain = drivetrain;
        this.telemetry = telemetry;

        timer = new Timer();

        addRequirements(scoringSubsystem, intakeSubsystem, drivetrain);
    }

    @Override
    public void initialize() {
        this.state = State.START;

        scoringSubsystem.setAction(ScoringAction.WAIT);
        scoringSubsystem.forceHood(false);
    }

    @Override
    public void execute() {
        switch (state) {
            case START:
                if (intakeSubsystem.hasNote()) {
                    state = State.DRIVE_TO_SCORING;
                } else {
                    state = State.DRIVE_TO_SOURCE;
                }
                break;
            case DRIVE_TO_SOURCE:
                // TODO: Implement automation of driving

                scoringSubsystem.setAction(ScoringAction.WAIT);
                if (false /*<-- are we at the source */) { // TODO: Check if we're close enough to
                    // the source to intake
                    state = State.INTAKE_NOTE;
                    timer.restart();
                }
                break;
            case INTAKE_NOTE:
                // TODO: Fully implement automation of intake

                intakeSubsystem.run(IntakeAction.INTAKE);

                if (intakeSubsystem.hasNote()) {
                    state = State.DRIVE_TO_SCORING;
                } else if (timer.get() > AutomatedTeleopConstants.intakeTimeoutSeconds) {
                    state = State.RETRY_INTAKE;
                    timer.restart();
                }
                break;
            case RETRY_INTAKE:
                // TODO: Implement automation of intake

                // Drive away from the source and wait for a certain timer

                // Once we are away and the time has elapsed, drive back
                if (timer.get()
                        > AutomatedTeleopConstants
                                .retryIntakeWaitSeconds /* && we are at target destination */) {
                    // Now that we've backed off, drive back and try to intake again
                    state = State.DRIVE_TO_SOURCE;
                }
                break;
            case DRIVE_TO_SCORING:
                // If we don't have a note, go get one
                // This will work for if we drop our note or for after we've shot
                if (!intakeSubsystem.hasNote()) {
                    // TODO: Use note vision to pick up a nearby note rather than going to source?
                    state = State.DRIVE_TO_SOURCE;
                }
                // TODO: Implement automation of driving

                scoringSubsystem.setAction(ScoringAction.SHOOT);
                break;
        }
    }
}
