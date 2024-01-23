package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraParams;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.localization.CameraIO;
import frc.robot.subsystems.localization.CameraIOPhoton;
import frc.robot.subsystems.localization.VisionLocalizer;
import frc.robot.subsystems.scoring.AimerIOSim;
import frc.robot.subsystems.scoring.AimerIOTalon;
import frc.robot.subsystems.scoring.HoodIOSim;
import frc.robot.subsystems.scoring.HoodIOVortex;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ShooterIOSim;
import frc.robot.subsystems.scoring.ShooterIOTalon;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RobotContainer {
    ScoringSubsystem scoringSubsystem;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController controller = new CommandXboxController(2);

    VisionLocalizer tagVision;

    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    Telemetry driveTelemetry = new Telemetry(DriveConstants.MaxSpeedMetPerSec);

    public RobotContainer() {
        configureBindings();
        configureSubsystems();
        configureModes();
    }

    // spotless:off
    private void configureBindings() {
        drivetrain.setDefaultCommand(
                new DriveWithJoysticks(
                        drivetrain,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX(),
                        () -> true,
                        () -> false));

        controller.a()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.INTAKE)));

        controller.b()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.AIM)));

        controller.x()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.SHOOT)))
                .onFalse(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.AIM)));

        controller.y()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.ENDGAME)))
                .onFalse(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.WAIT)));
        
        controller.back()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.AMP_SCORE)));

        controller.start()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.WAIT)));
    } // spotless:on

    private void configureModes() {}

    public void configureSubsystems() {
        if (Constants.currentMode == Constants.Mode.SIM) {
            drivetrain.seedFieldRelative(
                    new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(driveTelemetry::telemeterize);

        switch (Constants.currentMode) {
            case REAL:
                scoringSubsystem =
                        new ScoringSubsystem(
                                new ShooterIOTalon(),
                                new AimerIOTalon(),
                                new HoodIOVortex(),
                                driveTelemetry::getFieldToRobot);

                List<CameraIO> realCameras = new ArrayList<>();
                for (CameraParams params : VisionConstants.cameras) {
                    realCameras.add(CameraIOPhoton.fromCameraParams(params));
                }
                tagVision = new VisionLocalizer(realCameras);
                break;
            case SIM:
                scoringSubsystem =
                        new ScoringSubsystem(
                                new ShooterIOSim(),
                                new AimerIOSim(),
                                new HoodIOSim(),
                                driveTelemetry::getFieldToRobot);

                tagVision = new VisionLocalizer(Collections.emptyList());
                break;
            case REPLAY:
        }

        tagVision.setCameraConsumer(
                (m) -> drivetrain.addVisionMeasurement(m.pose(), m.timestamp(), m.variance()));
        tagVision.setFieldToRobotSupplier(driveTelemetry::getFieldToRobot);
    }
}
