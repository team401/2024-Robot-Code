package frc.robot.subsystems.scoring;

import java.time.Instant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class NoteSimSubsystem {
    
    
    private Timer timeSinceLaunch;

    private double lastRecordedTime;
    private Translation3d lastVelocity;
    private Pose3d lastNotePos;
    
    private boolean launching;

    private CommandSwerveDrivetrain drivetrain;
    private ScoringSubsystem scoring;

    private Command launchCommand;

    private static final Transform3d launcherTransform =
      new Transform3d(0.35, 0, 0.8, new Rotation3d(0.0, Units.degreesToRadians(-55.0), 0.0));
    
    public NoteSimSubsystem(CommandSwerveDrivetrain drivetrain, ScoringSubsystem scoring) {
        this.drivetrain = drivetrain;
        this.scoring = scoring;
    }

    public Command launch(Translation3d launchVelocity) {
        launching = true;
        lastVelocity = launchVelocity;
        lastRecordedTime = 0;
        lastNotePos = new Pose3d(drivetrain.getState().Pose).transformBy(launcherTransform);
        timeSinceLaunch.reset();
        timeSinceLaunch.start();

        launchCommand = new InstantCommand(() -> calculateTrajectory());

        return launchCommand.repeatedly();
    }

    private void calculateTrajectory() {
        //x = vnot*t - 1/2gt^2
        double deltaTime = timeSinceLaunch.get() - lastRecordedTime;

        double newZPosition = lastNotePos.getZ() + lastVelocity.getZ() * deltaTime - 1/2 * 9.8 * Math.pow(lastRecordedTime,2);
        double newYPosition = lastNotePos.getY() + lastVelocity.getY() * deltaTime;
        double newXPosition = lastNotePos.getX() + lastVelocity.getX() * deltaTime;

        double newVelocityZ = lastVelocity.getZ() - 9.8 * deltaTime;

        lastRecordedTime = timeSinceLaunch.get();
        lastNotePos = new Pose3d(newXPosition, newYPosition, newZPosition, lastNotePos.getRotation());
        lastVelocity = new Translation3d(lastVelocity.getX(), lastVelocity.getY(), newVelocityZ);

        if (newZPosition <= 0) launchCommand.end(true);
    }  

    public Pose3d getNotePosition() {
        return lastNotePos;
    }
}
