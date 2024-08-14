package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class NoteDetection extends SubsystemBase {

    private PhotonCamera colorCamera = new PhotonCamera("note-detector");
    private Supplier<Pose2d> robotPose;

    public NoteDetection(Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose;
        SmartDashboard.putBoolean("running", true);
    }

    private Pose2d getNotePoseFromTarget(PhotonTrackedTarget target) {
        double transformWidth = 1 * 78; // = distance at setpoint / width at setpoint
        double noteWidth = 0.1;
        // w of real note = 35 cm
        // .405 * 74 * 35 = d * wd

        List<TargetCorner> corners = target.getMinAreaRectCorners();

        double[][] cornersCoordinates = new double[4][2];
        SmartDashboard.putNumber("corners", corners.size());
        SmartDashboard.putNumber("area", target.getArea());
        for (int i = 0; i < corners.size(); i++) {
            cornersCoordinates[i][0] = corners.get(i).x;
            cornersCoordinates[i][1] = corners.get(i).y;
            SmartDashboard.putNumber("coordinate" + (i) + "x", cornersCoordinates[i][0]);
            SmartDashboard.putNumber("coordinate" + (i) + "y", cornersCoordinates[i][1]);
        }

        double width = cornersCoordinates[1][0] - cornersCoordinates[0][0];
        double distanceForward = transformWidth / width;
        // alternatively: instead of using magnification maybe try using just y instead?
        SmartDashboard.putNumber("distanceForward", distanceForward);
        // THEORETICALLY WORKING

        double widthShift = 0.180 * 199 / 2;
        double distanceSide =
                widthShift
                        * noteWidth
                        / ((cornersCoordinates[1][0] + cornersCoordinates[0][0]) / 2 - 160)
                        / distanceForward;
        SmartDashboard.putNumber("distanceSize", distanceSide);

        double distance =
                Math.sqrt(distanceForward * distanceForward + distanceSide * distanceSide);
        double angle = Math.tanh(distanceSide / distanceForward);

        Rotation2d fromForward = robotPose.get().getRotation().plus(new Rotation2d(angle));

        SmartDashboard.putNumber("width", width);
        SmartDashboard.putNumber(
                "centerx", (cornersCoordinates[1][0] + cornersCoordinates[0][0]) / 2);
        SmartDashboard.putNumber(
                "centery", (cornersCoordinates[1][1] + cornersCoordinates[0][1]) / 2);

        return new Pose2d(
                robotPose.get().getX() + Math.cos(fromForward.getRadians()) * distance,
                robotPose.get().getY() + Math.sin(fromForward.getRadians()) * distance,
                fromForward);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("hello", true);
        var cameraResult = colorCamera.getLatestResult();
        if (cameraResult.hasTargets()) {
            Pose2d notePose = getNotePoseFromTarget(cameraResult.getBestTarget());
            Logger.recordOutput("Note Position", notePose);
            SmartDashboard.putBoolean("target found", true);
        }
    }
}
