package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class NoteSimSubsystem {

    private Timer timeSinceLaunch;

    private double lastRecordedTime;
    private Translation3d lastVelocity;
    private Pose3d lastNotePos;

    public boolean launching;

    private CommandSwerveDrivetrain drivetrain;
    private ScoringSubsystem scoring;

    private Command launchCommand;

    private Pose3d speaker;

    private static final Transform3d launcherTransform =
            new Transform3d(0.35, 0, 0.8, new Rotation3d(0.0, Units.degreesToRadians(-55.0), 0.0));

    public NoteSimSubsystem(CommandSwerveDrivetrain drivetrain, ScoringSubsystem scoring) {
        this.drivetrain = drivetrain;
        this.scoring = scoring;

        timeSinceLaunch = new Timer();

        speaker =
                new Pose3d(
                        Constants.FieldConstants.fieldToRedSpeaker.getX(),
                        Constants.FieldConstants.fieldToRedSpeaker.getY(),
                        2,
                        new Rotation3d());
    }

    public Command launch(Translation3d launchVelocity) {
        SmartDashboard.putString("AHHH", ":D");
        return new ScheduleCommand( // Branch off and exit immediately
                Commands.defer(
                                () -> {
                                    launching = true;
                                    lastVelocity = launchVelocity;
                                    lastRecordedTime = 0;
                                    lastNotePos =
                                            new Pose3d(drivetrain.getState().Pose)
                                                    .transformBy(launcherTransform);
                                    timeSinceLaunch.reset();
                                    timeSinceLaunch.start();
                                    return Commands.run(
                                                    () -> {
                                                        SmartDashboard.putBoolean("??", true);
                                                        calculateTrajectory();
                                                        getNotePosition();
                                                    })
                                            .until(() -> timeSinceLaunch.hasElapsed(10));
                                },
                                Set.of())
                        .ignoringDisable(true));

        /*
        launching = true;
        lastVelocity = launchVelocity;
        lastRecordedTime = 0;
        lastNotePos = new Pose3d(drivetrain.getState().Pose).transformBy(launcherTransform);
        timeSinceLaunch.reset();
        timeSinceLaunch.start();

        return Commands.run(
                        () -> {
                            SmartDashboard.putBoolean("??", true);
                            calculateTrajectory();
                            getNotePosition();
                        })
                .ignoringDisable(true)
                .until(
                        () ->
                                lastNotePos.getZ() <= 0
                                        || lastNotePos
                                                        .minus(speaker)
                                                        .getTranslation()
                                                        .getDistance(new Translation3d(0, 0, 0))
                                                <= 0.5);*/
    }

    private void calculateTrajectory() {
        // x = vnot*t - 1/2gt^2
        double deltaTime = timeSinceLaunch.get() - lastRecordedTime;

        double newZPosition =
                lastNotePos.getZ()
                        + lastVelocity.getZ() * deltaTime
                        - 1 / 2 * 9.8 * Math.pow(lastRecordedTime, 2);
        double newYPosition = lastNotePos.getY() + lastVelocity.getY() * deltaTime;
        double newXPosition = lastNotePos.getX() + lastVelocity.getX() * deltaTime;

        double newVelocityZ = lastVelocity.getZ() - 9.8 * deltaTime;

        lastRecordedTime = timeSinceLaunch.get();
        lastNotePos =
                new Pose3d(newXPosition, newYPosition, newZPosition, lastNotePos.getRotation());
        lastVelocity = new Translation3d(lastVelocity.getX(), lastVelocity.getY(), newVelocityZ);
    }

    public Pose3d getNotePosition() {
        Logger.recordOutput("Note", lastNotePos);
        SmartDashboard.putNumber("Height", lastNotePos.getZ());
        return lastNotePos;
    }

    public boolean withinRange(Pose3d object, Pose3d withinCenter, double radius) {
        double dX = object.getX() - withinCenter.getY();
        double dY = object.getY() - withinCenter.getY();
        double dZ = object.getZ() - withinCenter.getZ();
        if (Math.sqrt(dX * dX + dY * dY + dZ * dZ) <= radius) return true;
        return false;
    }

    public boolean withinRectangle(Pose3d object, int[][] points) {

        return false; // in progress, unsure if there's an easy way to do pip for 3d or if there's
        // an existing library or something
    }
}
