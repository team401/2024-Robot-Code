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

    private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    private static Supplier<Double> shotRPMSupplier = () -> 0.0;
    private static Supplier<Double> hoodAngleSupplier = () -> 0.0;

    private static Pose3d lastPose = new Pose3d(100, 100, 100, new Rotation3d());

    public static void setSuppliers(
            Supplier<Pose2d> robotSupplier,
            Supplier<Double> rpmSupplier,
            Supplier<Double> hoodSupplierRad) {
        robotPoseSupplier = robotSupplier;
        hoodAngleSupplier = hoodSupplierRad;
        shotRPMSupplier = rpmSupplier;
    }

    // flywheels are 3 inches
    // spotless:off
    public static Command shoot() {
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
                                            .until(() -> lastPose.getZ() < 0)
                                            .finallyDo(
                                                    () -> {
                                                        Logger.recordOutput(
                                                                "NoteVisualizer", new Pose3d[] {});
                                                    });
                                },
                                Set.of())
                        .ignoringDisable(true));
    }// spotless:on

    public boolean withinRange(Pose3d object, Pose3d withinCenter, double radius) {
        double dX = object.getX() - withinCenter.getY();
        double dY = object.getY() - withinCenter.getY();
        double dZ = object.getZ() - withinCenter.getZ();
        if (Math.sqrt(dX * dX + dY * dY + dZ * dZ) <= radius) return true;
        return false;
    }

    /*public boolean inSpeaker() {
        boolean isRed = DriverStation.getAlliance().isPresent()
                          && DriverStation.getAlliance().get().equals(Alliance.Red);
        double[][] speakerOpening = {{16.317, 5, 1.98}, {16.317, 5.55, 1.98}, {16.317, 5, 2.11}, {16.317, 5.55, 2.11}};
        double[][] speakerRoof = {{16.317, 5, 2.11}, {16.317, 5.55, 2.11}, {20, 5, 2.5}, {20, 5.55, 2.5}};
        if ()

                      //new Pose3d(isRed ? redSpeaker : blueSpeaker, startPose.getRotation());
        return false; // in progress, unsure if there's an easy way to do pip for 3d or if there's
        // an existing library or something
    }*/
}
