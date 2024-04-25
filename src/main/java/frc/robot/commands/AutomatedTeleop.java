package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutomatedTeleopConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeAction;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import frc.robot.telemetry.Telemetry;
import frc.robot.utils.notesimulator.NoteManager;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutomatedTeleop extends Command {
    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;
    CommandSwerveDrivetrain drivetrain;

    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();

    Telemetry telemetry;

    private enum State {
        START,
        DRIVE_TO_SOURCE,
        ACQUIRE_NOTE,
        DRIVE_TO_SPEAKER,
        SHOOT_NOTE,
    }

    private State state;

    public AutomatedTeleop(
            ScoringSubsystem scoringSubsystem,
            IntakeSubsystem intakeSubsystem,
            CommandSwerveDrivetrain drivetrain,
            Telemetry telemetry,
            Supplier<Pose2d> poseSupplier) {
        this.scoringSubsystem = scoringSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.drivetrain = drivetrain;
        this.telemetry = telemetry;

        this.poseSupplier = poseSupplier;

        if (scoringSubsystem != null) {
            addRequirements(scoringSubsystem);
        }
        if (intakeSubsystem != null) {
            addRequirements(intakeSubsystem);
        }
        if (drivetrain != null) {
            addRequirements(drivetrain);
        }
    }

    @Override
    public void initialize() {
        this.state = State.START;

        scoringSubsystem.setAction(ScoringAction.WAIT);
        scoringSubsystem.forceHood(false);
    }

    private double findDistanceToSource() {
        // NOTE: Leave one of the following lines commented out depending on the scenario:

        // Uncomment this line for actual production code:
        // Pose2d sourcePose = AllianceUtil.getPoseAgainstSource();

        // Uncomment this line for testing with shop source
        Pose2d sourcePose = FieldConstants.robotAgainstShopSource;

        Pose2d robotPose = poseSupplier.get();
        double distancetoGoal =
                Math.sqrt(
                        Math.pow(Math.abs(robotPose.getX() - sourcePose.getX()), 2)
                                + Math.pow(Math.abs(robotPose.getY() - sourcePose.getY()), 2));
        return distancetoGoal;
    }

    @Override
    public void execute() {
        Logger.recordOutput("automatedTeleop/State", state.toString());
        switch (state) {
            case START:
                if (intakeSubsystem.hasNote()) {
                    state = State.DRIVE_TO_SPEAKER;
                } else {
                    state = State.DRIVE_TO_SOURCE;
                }
                break;
            case DRIVE_TO_SOURCE:
                drivetrain.driveToSource();

                scoringSubsystem.setAction(ScoringAction.WAIT);
                // Once robot reaches the source, stop driving and try to acquire a note
                if (findDistanceToSource()
                        < AutomatedTeleopConstants.sourceRangeMeters /* || note detected */) {
                    state = State.ACQUIRE_NOTE;
                }
                break;
            case ACQUIRE_NOTE:
                // TODO: Fully implement automation of intake

                drivetrain.stopDriveToPose();
                intakeSubsystem.run(IntakeAction.INTAKE);

                NoteManager.intake();

                if (intakeSubsystem.hasNote()
                        || (Constants.currentMode == Constants.Mode.SIM
                                && NoteManager.noteInRobot())) {
                    state = State.DRIVE_TO_SPEAKER;
                }
                break;
            case DRIVE_TO_SPEAKER:
                // If we don't have a note, go get one
                // This will work for if we drop our note or for after we've shot
                if (!intakeSubsystem.hasNote()) {
                    // TODO: Use note vision to pick up a nearby note rather than going to source?
                    state = State.DRIVE_TO_SOURCE;
                }

                drivetrain.driveToSpeaker();
                scoringSubsystem.setAction(ScoringAction.AIM);

                // Once we are in range, shoot
                if (scoringSubsystem.findDistanceToGoal()
                        < AutomatedTeleopConstants.shootRangeMeters) {
                    state = State.SHOOT_NOTE;
                }
                break;
            case SHOOT_NOTE:
                if (!intakeSubsystem.hasNote()) {
                    state = State.DRIVE_TO_SOURCE;
                }

                drivetrain.driveToSpeaker();
                scoringSubsystem.setAction(ScoringAction.SHOOT);

                break;
        }
    }
}
