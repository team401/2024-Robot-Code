package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class AllianceUtil {

    public static Translation2d getFieldToSpeaker() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    Logger.recordOutput("Field/speaker", FieldConstants.fieldToBlueSpeaker);
                    return FieldConstants.fieldToBlueSpeaker;
                case Red:
                    Logger.recordOutput("Field/speaker", FieldConstants.fieldToRedSpeaker);
                    return FieldConstants.fieldToRedSpeaker;
            }
        }
        return FieldConstants.fieldToRedSpeaker;
    }

    public static Rotation2d getAmpHeading() {
        Logger.recordOutput("Field/amp", FieldConstants.ampHeading);
        return FieldConstants.ampHeading;
    }

    public static Pose2d getPoseAgainstSpeaker() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.robotAgainstBlueSpeaker;
                case Red:
                    return FieldConstants.robotAgainstRedSpeaker;
            }
        }
        return FieldConstants.robotAgainstRedSpeaker;
    }

    public static Pose2d getPoseAgainstSpeakerLeft() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.robotAgainstBlueSpeakerLeft;
                case Red:
                    return FieldConstants.robotAgainstRedSpeakerLeft;
            }
        }
        return FieldConstants.robotAgainstRedSpeakerLeft;
    }

    public static Pose2d getPoseAgainstSpeakerRight() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.robotAgainstBlueSpeakerRight;
                case Red:
                    return FieldConstants.robotAgainstRedSpeakerRight;
            }
        }
        return FieldConstants.robotAgainstRedSpeakerRight;
    }

    public static Pose2d getPoseAgainstPodium() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.robotAgainstBluePodium;
                case Red:
                    return FieldConstants.robotAgainstRedPodium;
            }
        }
        return FieldConstants.robotAgainstRedPodium;
    }

    public static Pose2d getPoseAgainstAmpZone() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.robotAgainstRedAmpZone;
                case Red:
                    return FieldConstants.robotAgainstBlueAmpZone;
            }
        }
        return FieldConstants.robotAgainstRedAmpZone;
    }

    public static Rotation2d getSourceHeading() {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    Logger.recordOutput("Field/source", FieldConstants.blueSourceHeading);
                    return FieldConstants.blueSourceHeading;
                case Red:
                    Logger.recordOutput("Field/source", FieldConstants.redSourceHeading);
                    return FieldConstants.redSourceHeading;
            }
        }
        return FieldConstants.redSourceHeading;
    }

    /** Returns whether the speaker is significantly to the robot's left */
    public static boolean isLeftOfSpeaker(double robotY, double tolerance) {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return robotY > FieldConstants.fieldToBlueSpeaker.getY() + tolerance;
                case Red:
                    return robotY < FieldConstants.fieldToRedSpeaker.getY() - tolerance;
            }
        }
        return robotY < FieldConstants.fieldToRedSpeaker.getY() - tolerance;
    }

    /** Returns whether the speaker is significantly to the robot's right */
    public static boolean isRightOfSpeaker(double robotY, double tolerance) {
        if (!DriverStation.getAlliance().isEmpty()) {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return robotY < FieldConstants.fieldToBlueSpeaker.getY() - tolerance;
                case Red:
                    return robotY > FieldConstants.fieldToRedSpeaker.getY() + tolerance;
            }
        }
        return robotY > FieldConstants.fieldToRedSpeaker.getY() + tolerance;
    }
}
