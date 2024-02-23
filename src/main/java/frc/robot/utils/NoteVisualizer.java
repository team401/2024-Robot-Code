// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {

    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    private Supplier<Double> shotRPMSupplier = () -> 0.0;
    private Supplier<Double> hoodAngleSupplier = () -> 0.0;

    private Pose3d lastPose = new Pose3d(100, 100, 100, new Rotation3d());

    public NoteVisualizer(
            Supplier<Pose2d> robotSupplier,
            Supplier<Double> rpmSupplier,
            Supplier<Double> hoodSupplierRad) {
        robotPoseSupplier = robotSupplier;
        hoodAngleSupplier = hoodSupplierRad;
        shotRPMSupplier = rpmSupplier;
    }

    // flywheels are 3 inches
    // spotless:off
    public Command shoot() {
        return new ScheduleCommand( // Branch off and exit immediately
                Commands.defer(
                                () -> {
                                    Pose3d startPose =
                                            new Pose3d(robotPoseSupplier.get())
                                                    .transformBy(new Transform3d(0.35, 0, 0.8, new Rotation3d()));
                                    Timer timeSinceLaunch = new Timer();
                                    timeSinceLaunch.start();
                                    double shotSpeed =
                                            shotRPMSupplier.get()
                                                    * 2
                                                    * Math.PI
                                                    / 60
                                                    * 0.5
                                                    * 1; // * 0.0381; // are the values on the
                                    // interpolate map correct?

                                    return Commands.run(
                                        () -> {
                                            lastPose =
                                                new Pose3d(
                                                    shotSpeed
                                                        * Math.cos(hoodAngleSupplier.get())
                                                        * Math.cos(robotPoseSupplier.get().getRotation().getRadians())
                                                        * timeSinceLaunch.get()
                                                    + startPose.getX(),
                                                    shotSpeed
                                                        * Math.cos(hoodAngleSupplier.get())
                                                        * Math.sin(robotPoseSupplier.get().getRotation().getRadians())
                                                        * timeSinceLaunch.get()
                                                    + startPose.getY(),
                                                    shotSpeed
                                                        * Math.sin(hoodAngleSupplier.get())
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
                                            .until(() -> inSpeaker() != 1)
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
    }// spotless:on

    private boolean withinRange(Pose3d object, Pose3d withinCenter, double radius) {
        double dX = object.getX() - withinCenter.getY();
        double dY = object.getY() - withinCenter.getY();
        double dZ = object.getZ() - withinCenter.getZ();
        if (Math.sqrt(dX * dX + dY * dY + dZ * dZ) <= radius) return true;
        return false;
    }

    private int inSpeaker() {
        /*boolean isRed = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get().equals(Alliance.Red);*/
        double[][] speakerOpening = {
            {16.08, 5, 1.98}, {16.08, 5.55, 1.98}, {16.51, 5, 2.11}, {16.51, 5.55, 2.11}
        };
        double[][] speakerRoof = {
            {16.51, 5, 2.11}, {16.51, 5.55, 2.11}, {16.08, 5, 2.5}, {16.08, 5.55, 2.5}
        };
        if (passesThroughRectangle(speakerOpening, lastPose)) return 2;
        if (passesThroughRectangle(speakerRoof, lastPose) || lastPose.getZ() < 0) return 0;

        // new Pose3d(isRed ? redSpeaker : blueSpeaker, startPose.getRotation());
        return 1; // in progress, unsure if there's an easy way to do pip for 3d or if there's
        // an existing library or something
    }

    // only works on rectangles parallel to z and tilted forward on x
    private boolean passesThroughRectangle(double[][] rectangle, Pose3d noteLocation) {
        double x = noteLocation.getX();
        double y = noteLocation.getY();
        double z = noteLocation.getZ();

        if (y > rectangle[0][1] && y < rectangle[3][1]) {
            double slope =
                    (rectangle[3][2] - rectangle[0][2]) / (rectangle[3][0] - rectangle[0][0]);
            double predictedZ = slope * (x - rectangle[3][0]) + rectangle[3][2];
            SmartDashboard.putNumber("difference from goal", Math.abs(predictedZ - z));
            if (Math.abs(predictedZ - z) < 0.1) return true;
        }
        return false;
    }
}
