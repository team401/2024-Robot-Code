package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.localization.VisionIOReal;
import frc.robot.subsystems.localization.VisionLocalizer;
import frc.robot.subsystems.scoring.AimerIOSim;
import frc.robot.subsystems.scoring.AimerIOTalon;
import frc.robot.subsystems.scoring.HoodIOSim;
import frc.robot.subsystems.scoring.HoodIOVortex;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ShooterIOSim;
import frc.robot.subsystems.scoring.ShooterIOTalon;
import frc.robot.utils.FieldFinder;
import org.littletonrobotics.junction.Logger;

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
        
        controller.rightBumper()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.AMP_AIM)));

        controller.start()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.WAIT)));
        
        controller.povUp()
                .onTrue(new InstantCommand(
                    () -> drivetrain.seedFieldRelative(
                        new Pose2d(new Translation2d(driveTelemetry.m_lastPose.getX(), driveTelemetry.m_lastPose.getY() + 0.1), Rotation2d.fromDegrees(90)))));
        
        controller.povDown()
                .onTrue(new InstantCommand(
                    () -> drivetrain.seedFieldRelative(
                        new Pose2d(new Translation2d(driveTelemetry.m_lastPose.getX(), driveTelemetry.m_lastPose.getY() - 0.1), Rotation2d.fromDegrees(90)))));

        controller.povLeft()
                .onTrue(new InstantCommand(
                    () -> drivetrain.seedFieldRelative(
                        new Pose2d(new Translation2d(driveTelemetry.m_lastPose.getX() - 0.1, driveTelemetry.m_lastPose.getY()), Rotation2d.fromDegrees(90)))));

        controller.povRight()
                .onTrue(new InstantCommand(
                    () -> drivetrain.seedFieldRelative(
                        new Pose2d(new Translation2d(driveTelemetry.m_lastPose.getX() + 0.1, driveTelemetry.m_lastPose.getY()), Rotation2d.fromDegrees(90)))));
    } // spotless:on

    private void configureModes() {}

    public void configureSubsystems() {
        switch (Constants.currentMode) {
            case REAL:
                scoringSubsystem =
                        new ScoringSubsystem(
                                new ShooterIOTalon(),
                                new AimerIOTalon(),
                                new HoodIOVortex(),
                                driveTelemetry::getFieldToRobot,
                                () ->
                                        VecBuilder.fill(
                                                driveTelemetry.getVelocityX(),
                                                driveTelemetry.getVelocityY()));

                tagVision = new VisionLocalizer(new VisionIOReal(VisionConstants.cameras));
                break;
            case SIM:
                drivetrain.seedFieldRelative(
                        new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));

                scoringSubsystem =
                        new ScoringSubsystem(
                                new ShooterIOSim(),
                                new AimerIOSim(),
                                new HoodIOSim(),
                                driveTelemetry::getFieldToRobot,
                                () ->
                                        VecBuilder.fill(
                                                driveTelemetry.getVelocityX(),
                                                driveTelemetry.getVelocityY()));

                // tagVision =
                //         new VisionLocalizer(
                //                 new VisionIOSim(
                //                         VisionConstants.cameras,
                // driveTelemetry::getFieldToRobot));
                break;
            case REPLAY:
                break;
        }

        drivetrain.registerTelemetry(driveTelemetry::telemeterize);
        Commands.run(driveTelemetry::logDataSynchronously).ignoringDisable(true).schedule();

        // tagVision.setCameraConsumer(
        //         (m) -> drivetrain.addVisionMeasurement(m.pose(), m.timestamp(), m.variance()));
        // tagVision.setFieldToRobotSupplier(driveTelemetry::getFieldToRobot);
    }

    public void robotPeriodic() {
        Logger.recordOutput(
                "localizer/whereAmI",
                FieldFinder.whereAmI(
                        driveTelemetry.getFieldToRobot().getTranslation().getX(),
                        driveTelemetry.getFieldToRobot().getTranslation().getY()));
        Logger.recordOutput(
                "localizer/willIHitBlueStage",
                FieldFinder.willIHitThis(
                        driveTelemetry.getFieldToRobot().getTranslation().getX(),
                        driveTelemetry.getFieldToRobot().getTranslation().getY(),
                        driveTelemetry.getVelocityX() * 5,
                        driveTelemetry.getVelocityY() * 5,
                        FieldFinder.FieldLocations.BLUE_STAGE));
        Logger.recordOutput(
                "localizer/trajectory",
                new Pose2d(
                        driveTelemetry.getFieldToRobot().getX() + driveTelemetry.getVelocityX() * 5,
                        driveTelemetry.getFieldToRobot().getY() + driveTelemetry.getVelocityY() * 5,
                        driveTelemetry.getFieldToRobot().getRotation()));
    }
}
