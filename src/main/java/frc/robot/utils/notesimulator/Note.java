package frc.robot.utils.notesimulator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Note {

    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    private Pose3d lastPose = new Pose3d(100, 100, 100, new Rotation3d());

    private boolean noteInRobot = false;

    public Note(Supplier<Pose2d> robotSupplier, Pose2d noteStartingPosition, boolean noteInRobot) {
        robotPoseSupplier = robotSupplier;

        this.noteInRobot = noteInRobot;
        if (!noteInRobot) {
            lastPose = new Pose3d(noteStartingPosition);
            Logger.recordOutput("NoteVisualizer", lastPose);
        }
    }

    public Note(Supplier<Pose2d> robotSupplier, boolean noteInRobot) {

        this(
                robotSupplier,
                new Pose2d(Math.random() * 10 + 5, Math.random() * 2 + 4, new Rotation2d()),
                noteInRobot);
    }

    public Note(Supplier<Pose2d> robotSupplier, Pose2d noteStartingPosition) {

        this(robotSupplier, noteStartingPosition, false);
    }

    public Note(Supplier<Pose2d> robotSupplier) {

        this(
                robotSupplier,
                new Pose2d(Math.random() * 10 + 5, Math.random() * 4 + 1, new Rotation2d()),
                false);
    }

    // flywheels are 3 inches
    // spotless:off
    public Command shoot(double aimRPM, double aimAngle) {
        if (noteInRobot) {
        return new ScheduleCommand( // Branch off and exit immediately
                Commands.defer(
                                () -> {
                                    Pose3d startPose =
                                            new Pose3d(robotPoseSupplier.get())
                                                    .transformBy(new Transform3d(0.35, 0, 0.8, new Rotation3d()));
                                    Timer timeSinceLaunch = new Timer();
                                    timeSinceLaunch.start();
                                    double shotSpeed =
                                            aimRPM
                                                    * 2
                                                    * Math.PI
                                                    / 60
                                                    * 0.5
                                                    * 2 * 0.0381; // are the values on the
                                    // interpolate map correct?

                                    return Commands.run(
                                        () -> {
                                            lastPose =
                                                new Pose3d(
                                                    shotSpeed
                                                        * Math.cos(aimAngle)
                                                        * Math.cos(robotPoseSupplier.get().getRotation().getRadians())
                                                        * timeSinceLaunch.get()
                                                    + startPose.getX(),
                                                    shotSpeed
                                                        * Math.cos(aimAngle)
                                                        * Math.sin(robotPoseSupplier.get().getRotation().getRadians())
                                                        * timeSinceLaunch.get()
                                                    + startPose.getY(),
                                                    shotSpeed
                                                        * Math.sin(aimAngle)
                                                        * timeSinceLaunch.get()
                                                    + startPose.getZ()
                                                    - 4.9 * Math.pow(timeSinceLaunch.get(),2),
                                                    startPose.getRotation());
                                            Logger.recordOutput(
                                                    "NoteVisualizer", lastPose);
                                            SmartDashboard.putNumber(
                                                    "angle",
                                                    robotPoseSupplier
                                                            .get()
                                                            .getRotation()
                                                            .getDegrees());
                                                    })
                                            .until(() -> lastPose != null && inSpeaker() != 1)
                                            .finallyDo(
                                                    () -> {
                                                        Logger.recordOutput(
                                                                "NoteVisualizer", new Pose3d[] {});
                                                        if (inSpeaker() == 2) SmartDashboard.putBoolean("inGoal", true);
                                                        else SmartDashboard.putBoolean("inGoal", false);
                                                    });
                                },
                                Set.of())
                        .ignoringDisable(true));
        }
        else return null;
    }// spotless:on

    public boolean intakeNote() {
        if (noteInRobot == false && robotWithinRange(1)) {
            noteInRobot = true;
            lastPose = null;
            Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
            return true;
        }
        return false;
    }

    public boolean inRobot() {
        return noteInRobot;
    }

    private boolean robotWithinRange(double radius) {
        double dX = lastPose.getX() - robotPoseSupplier.get().getX();
        double dY = lastPose.getY() - robotPoseSupplier.get().getY();
        if (Math.sqrt(dX * dX + dY * dY) <= radius) return true;
        return false;
    }

    private int inSpeaker() {
        boolean isRed =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get().equals(Alliance.Red);
        double[][] redSpeakerOpening = {
            {16.08, 5, 1.98}, {16.08, 5.55, 1.98}, {16.51, 5, 2.11}, {16.51, 5.55, 2.11}
        };
        double[][] blueSpeakerOpening = {
            {0.43, 5, 1.98}, {0.43, 5.55, 1.98}, {0, 5, 2.11}, {0, 5.55, 2.11}
        };
        double[][] redSpeakerRoof = {
            {16.51, 5, 2.11}, {16.51, 5.55, 2.11}, {16.08, 5, 2.5}, {16.08, 5.55, 2.5}
        };
        double[][] blueSpeakerRoof = {
            {0, 5, 2.11}, {0, 5.55, 2.11}, {0.43, 5, 2.5}, {0.43, 5.55, 2.5}
        };
        if (passesThroughRectangle(isRed ? redSpeakerOpening : blueSpeakerOpening, lastPose))
            return 2;
        if (passesThroughRectangle(isRed ? redSpeakerRoof : blueSpeakerRoof, lastPose)
                || lastPose.getZ() < 0) return 0;

        // new Pose3d(isRed ? redSpeaker : blueSpeaker, startPose.getRotation());
        return 1; // in progress, unsure if there's an easy way to do pip for 3d or if there's
        // an existing library or something
    }

    // only works on rectangles parallel to z and tilted forward on x
    private boolean passesThroughRectangle(double[][] rectangle, Pose3d noteLocation) {
        double x = noteLocation.getX();
        double y = noteLocation.getY();
        double z = noteLocation.getZ();

        if (y > Math.min(rectangle[0][1], rectangle[3][1])
                && y < Math.max(rectangle[0][1], rectangle[3][1])
                && z > Math.min(rectangle[0][2], rectangle[3][2])
                && z < Math.max(rectangle[0][2], rectangle[3][2])) {
            double slope =
                    (rectangle[3][2] - rectangle[0][2]) / (rectangle[3][0] - rectangle[0][0]);
            double predictedZ = slope * (x - rectangle[3][0]) + rectangle[3][2];
            SmartDashboard.putNumber("difference from goal", Math.abs(predictedZ - z));
            if (Math.abs(predictedZ - z) < 0.1) return true;
        }
        return false;
    }
}
