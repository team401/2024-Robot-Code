package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraParams;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {

    private VisionSystemSim visionSim = new VisionSystemSim("main");

    private List<CameraIO> cameras = new ArrayList<>();
    private List<CameraIOInputsAutoLogged> inputs = new ArrayList<>();

    private Supplier<SwerveModuleState[]> getModuleStates;
    private Pose2d latestOdometryPose;
    private SwerveModulePosition[] lastModulePositions =
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private Timer dtTimer = new Timer();

    public VisionIOSim(List<CameraParams> params, Supplier<SwerveModuleState[]> getModuleStates) {
        this.getModuleStates = getModuleStates;

        visionSim.addAprilTags(VisionConstants.fieldLayout);

        var pose = DriveConstants.initialPose;
        latestOdometryPose =
                new Pose2d(
                        pose.getX(),
                        pose.getY(),
                        Rotation2d.fromRadians(pose.getRotation().getRadians()));

        for (CameraParams param : params) {
            cameras.add(CameraIOPhoton.fromSimCameraParams(param, visionSim, true));
            inputs.add(new CameraIOInputsAutoLogged());
        }
    }

    public List<CameraIOInputsAutoLogged> getInputs() {
        updateOdometry();
        visionSim.update(latestOdometryPose);
        SmartDashboard.putData("PhotonSimField", visionSim.getDebugField());

        for (int i = 0; i < cameras.size(); i++) {
            cameras.get(i).updateInputs(inputs.get(i));

            Logger.processInputs("Vision/Camera" + i, inputs.get(i));
        }

        return inputs;
    }

    private void updateOdometry() {
        SwerveModulePosition[] deltas = new SwerveModulePosition[4];
        SwerveModuleState[] states = getModuleStates.get();

        double dt = dtTimer.get();
        dtTimer.reset();
        dtTimer.start();

        for (int i = 0; i < states.length; i++) {
            deltas[i] =
                    new SwerveModulePosition(
                            states[i].speedMetersPerSecond * dt
                                    - lastModulePositions[i].distanceMeters,
                            Rotation2d.fromRadians(
                                    states[i]
                                            .angle
                                            .minus(lastModulePositions[i].angle)
                                            .getRadians()));
        }

        Twist2d twist = TunerConstants.kinematics.toTwist2d(deltas);
        latestOdometryPose = latestOdometryPose.exp(twist);

        Logger.recordOutput("Vision/GroundTruth", latestOdometryPose);
    }
}
