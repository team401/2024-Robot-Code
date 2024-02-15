package frc.robot.subsystems.scoring;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

public class NoteSimSubsystem {
    
    
    private Timer timeSinceLaunch;

    private double lastRecordedTime;
    private Translation3d lastVelocity;
    private Pose3d lastNotePos;
    
    
    public NoteSimSubsystem(Pose2d startingPose) {
        lastNotePos = new Pose3d(startingPose);
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
        
    }
}
