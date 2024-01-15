package frc.robot.subsystems.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.PoseKalmanFilter;
import frc.robot.utils.PoseEstimator;

public class Localizer extends SubsystemBase {
    private final List<CameraIO> cameras;
    private final List<CameraIOInputsAutoLogged> cameraInputs = new ArrayList<>();

    private Supplier<Twist2d> odometrySupplier;

    private PoseEstimator poseEstimator = new PoseEstimator();

    public Localizer(List<CameraIO> cameras) {
        this.cameras = cameras;
        for (int i = 0; i < cameras.size(); i++) {
            cameraInputs.add(new CameraIOInputsAutoLogged());
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < cameras.size(); i++) {
            cameras.get(i).updateInputs(cameraInputs.get(i));
            
            var inputs = cameraInputs.get(i);

            poseEstimator.addCameraMeasurement(
                inputs.latestFieldToRobot,
                cameraUncertainty(inputs.averageTagDistanceM),
                inputs.latestTimestampSeconds);
        }

        poseEstimator.addOdometryMeasurement(
            odometrySupplier.get(),
            VisionConstants.driveUncertainty,
            Timer.getFPGATimestamp());

        Logger.recordOutput("FieldToRobot", getFieldToRobot());
    }

    public Pose2d getFieldToRobot() {
        return poseEstimator.getFieldToRobot();
    }

    public void setOdometrySupplier(Supplier<Twist2d> odometrySupplier) {
        this.odometrySupplier = odometrySupplier;
    }

    // I love 31-character symbol names
    private Matrix<N3, N1> cameraUncertainty(double averageTagDistanceM) {
        /*
         * On this year's field, Apriltags are arranged into rough 'corridors' between the stage and
         * speaker, and a central 'dessert,' where few tags can be found. It follows that we should
         * determine the variance of our camera mesurements based on that.
         */
        if (averageTagDistanceM < 2.0 && this.robotInMidField()) {
            return VisionConstants.lowCameraUncertainty;
        } else {
            return VisionConstants.highCameraUncertainty;
        }
    }

    private boolean robotInMidField() {
        // TODO
        return false;
    }
}
