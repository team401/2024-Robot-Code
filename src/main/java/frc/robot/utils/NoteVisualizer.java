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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
    private static final Transform3d launcherTransform =
            new Transform3d(0.35, 0, 0.8, new Rotation3d());
    private static double shotSpeed = 5.0; // Meters per sec
    private static double shotAngleRad = Math.PI / 6;
    private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

    public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        robotPoseSupplier = supplier;
    }

    public static Command shoot() {
        return new ScheduleCommand( // Branch off and exit immediately
                Commands.defer(
                                () -> {
                                    Pose3d startPose =
                                            new Pose3d(robotPoseSupplier.get())
                                                    .transformBy(launcherTransform);
                                    boolean isRed =
                                            DriverStation.getAlliance().isPresent()
                                                    && DriverStation.getAlliance()
                                                            .get()
                                                            .equals(Alliance.Red);
                                    Timer timeSinceLaunch = new Timer();
                                    timeSinceLaunch.start();
                                    return Commands.run(
                                                    () -> {
                                                        Logger.recordOutput(
                                                                "NoteVisualizer",
                                                                new Pose3d(
                                                                        shotSpeed
                                                                                        * Math.cos(
                                                                                                shotAngleRad)
                                                                                        * Math.sin(
                                                                                                startPose
                                                                                                        .getRotation()
                                                                                                        .getAngle())
                                                                                        * timeSinceLaunch
                                                                                                .get()
                                                                                + startPose.getX(),
                                                                        shotSpeed
                                                                                        * Math.cos(
                                                                                                shotAngleRad)
                                                                                        * Math.cos(
                                                                                                startPose
                                                                                                        .getRotation()
                                                                                                        .getAngle())
                                                                                        * timeSinceLaunch
                                                                                                .get()
                                                                                + startPose.getY(),
                                                                        shotSpeed
                                                                                        * Math.sin(
                                                                                                shotAngleRad)
                                                                                        * timeSinceLaunch
                                                                                                .get()
                                                                                + startPose.getZ()
                                                                                - 1
                                                                                        / 2
                                                                                        * 9.8
                                                                                        * Math.pow(
                                                                                                timeSinceLaunch
                                                                                                        .get(),
                                                                                                2),
                                                                        startPose.getRotation()));
                                                    })
                                            .until(() -> timeSinceLaunch.hasElapsed(10))
                                            .finallyDo(
                                                    () -> {
                                                        Logger.recordOutput(
                                                                "NoteVisualizer", new Pose3d[] {});
                                                    });
                                },
                                Set.of())
                        .ignoringDisable(true));
    }
}
