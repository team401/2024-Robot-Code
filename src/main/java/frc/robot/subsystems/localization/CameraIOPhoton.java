package frc.robot.subsystems.localization;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.VisionConstants;

public class CameraIOPhoton implements CameraIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private double latestTimestampSeconds = 0.0;

    public CameraIOPhoton(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(
            VisionConstants.fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        if (!camera.isConnected()) {
            Logger.recordOutput("Vision/" + camera.getName() + "/Connected", false);
            return;
        }
        Logger.recordOutput("Vision/" + camera.getName() + "/Connected", true);

        PhotonPipelineResult result = camera.getLatestResult();
        if (result.getTimestampSeconds() == latestTimestampSeconds) {
            return;
        }
        latestTimestampSeconds = result.getTimestampSeconds();
        Optional<EstimatedRobotPose> photonPose = poseEstimator.update(result);

        photonPose.filter(CameraIOPhoton::filterPhotonPose);

        photonPose.ifPresent((pose) -> {
            inputs.latestFieldToRobot = pose.estimatedPose.toPose2d();
            inputs.latestTimestampSeconds = this.latestTimestampSeconds;
            inputs.averageTagDistanceM = calculateAverageTagDistance(pose);
        });
    }

    private static boolean filterPhotonPose(EstimatedRobotPose photonPose) {
        if (photonPose.targetsUsed.size() == 1) {
            // if we only see one tag, filter strictly
            double ambiguity = photonPose.targetsUsed.get(0).getPoseAmbiguity();
            if (ambiguity > VisionConstants.singleTagAmbiguityCutoff || ambiguity == -1) {
                return false;
            }
        }

        Pose3d pose = photonPose.estimatedPose;
        // check that the pose isn't insane
        if (pose.getZ() > 1 || pose.getZ() < 0.1) {
            return false;
        }

        return true;
    }

    private static double calculateAverageTagDistance(EstimatedRobotPose pose) {
        double distance = 0.0;
        for (PhotonTrackedTarget target : pose.targetsUsed) {
            distance += target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
        }
        distance /= pose.targetsUsed.size();
        
        return distance;
    }
}
