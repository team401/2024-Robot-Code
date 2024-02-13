package frc.robot.utils;

// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N2;

/** Geometry utilities for working with translations, rotations, transforms, and poses. */
public class GeomUtil {
    /**
     * Creates a pure translating transform
     *
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d translationToTransform(Translation2d translation) {
        return new Transform2d(translation, new Rotation2d());
    }

    /**
     * Creates a pure translating transform
     *
     * @param x The x componenet of the translation
     * @param y The y componenet of the translation
     * @return The resulting transform
     */
    public static Transform2d translationToTransform(double x, double y) {
        return new Transform2d(new Translation2d(x, y), new Rotation2d());
    }

    /**
     * Creates a pure rotating transform
     *
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d rotationToTransform(Rotation2d rotation) {
        return new Transform2d(new Translation2d(), rotation);
    }

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform2d poseToTransform(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose2d transformToPose(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Creates a pure translated pose
     *
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d translationToPose(Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    /**
     * Creates a translation from the x and y of a pose, ignoring rotation
     *
     * @param pose The pose to create the translation with
     * @return The resulting translation
     */
    public static Translation2d poseToTranslation(Pose2d pose) {
        return new Translation2d(pose.getX(), pose.getY());
    }

    /**
     * Creates a pure rotated pose
     *
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d rotationToPose(Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Multiplies a twist by a scaling factor
     *
     * @param twist The twist to multiply
     * @param factor The scaling factor for the twist components
     * @return The new twist
     */
    public static Twist2d multiplyTwist(Twist2d twist, double factor) {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }

    /**
     * Converts a Pose3d to a Transform3d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform3d pose3dToTransform3d(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose3d transform3dToPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Y). chain
     *
     * @param transform The original translation
     * @return The resulting translation
     */
    public static Translation2d translation3dTo2dXY(Translation3d translation) {
        return new Translation2d(translation.getX(), translation.getY());
    }

    /**
     * Converts a Translation3d to a Translation2d by extracting two dimensions (X and Z). chain
     *
     * @param transform The original translation
     * @return The resulting translation
     */
    public static Translation2d translation3dTo2dXZ(Translation3d translation) {
        return new Translation2d(translation.getX(), translation.getZ());
    }

    // TODO: remove fVTPTest (temp function to debug findVelocityTowardPoint)
    public static double fVTPTest(
            Vector<N2> velocity,
            Translation2d currentTranslation,
            Translation2d targetTranslation) {

        // Vector from current to target (translate target so that current is now 0, 0)
        Translation2d currentToTarget = targetTranslation.minus(currentTranslation);
        // Convert it to a vector so we can take norm of it later
        Vector<N2> currentToTargetVec =
                VecBuilder.fill(currentToTarget.getX(), currentToTarget.getY());

        // Get the angle of the vector from robot to speaker
        Rotation2d speakerAngle =
                Rotation2d.fromRadians(Math.atan2(currentToTarget.getY(), currentToTarget.getX()));

        // Get the angle of the velocity vector of the robot
        Rotation2d velocityAngle =
                Rotation2d.fromRadians(Math.atan2(velocity.get(1, 0), velocity.get(0, 0)));

        // Calculate theta, the angle between the speakerAngle and the current velocity angle
        Rotation2d theta = velocityAngle.minus(speakerAngle);
        // Project the velocity vector onto the speaker vector with ||u|| * cos(theta)
        // Where u is the vector toward the goal
        double velocityToGoal = currentToTargetVec.norm() * theta.getCos();

        return velocityAngle.getDegrees();
    }

    public static double findVelocityTowardPoint(
            Vector<N2> velocity,
            Translation2d currentTranslation,
            Translation2d targetTranslation) {
        // Calculate the velocity to the goal based on this formula
        // https://web.ma.utexas.edu/users/m408m/Display12-3-4.shtml

        // Vector from current to target (translate target so that current is now 0, 0)
        Translation2d currentToTarget = targetTranslation.minus(currentTranslation);
        // Convert it to a vector so we can take norm of it later
        Vector<N2> currentToTargetVec =
                VecBuilder.fill(currentToTarget.getX(), currentToTarget.getY());

        // Get the angle of the vector from current to target
        Rotation2d targetAngle =
                Rotation2d.fromRadians(Math.atan2(currentToTarget.getY(), currentToTarget.getX()));

        // Get the angle of the velocity vector
        Rotation2d velocityAngle =
                Rotation2d.fromRadians(Math.atan2(velocity.get(1, 0), velocity.get(0, 0)));

        // Calculate theta, the angle between the speakerAngle and the current velocity angle
        Rotation2d theta = velocityAngle.minus(targetAngle);
        // Project the velocity vector onto the speaker vector with ||u|| * cos(theta)
        // Where u is the vector toward the goal
        double velocityToGoal = currentToTargetVec.norm() * theta.getCos();

        return velocityToGoal;
    }
}
