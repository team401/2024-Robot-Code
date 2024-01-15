package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseEstimator {
    //TODO: figure out how a kalman filter works

    public PoseEstimator() {}

    public void addCameraMeasurement(Pose2d pose, Matrix<N3, N1> uncertainty, double timestamp) {}

    public void addOdometryMeasurement(Twist2d delta, Matrix<N3, N1> uncertainty, double timestamp) {}

    public Pose2d getFieldToRobot() {return  null;}
}
