package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.scoring.AimerIO;
import frc.robot.subsystems.scoring.AimerIOSim;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ShooterIO;
import frc.robot.subsystems.scoring.ShooterIOSim;

public class RobotContainer {
    ScoringSubsystem scoringSubsystem;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController controller = new CommandXboxController(2);

    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    Telemetry driveTelemetry = new Telemetry(DriveConstants.MaxSpeedMetPerSec);

    public RobotContainer() {
        configureBindings();
        configureSubsystems();
        configureModes();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain,
        () -> controller.getLeftY(),
        () -> controller.getLeftX(),
        () -> controller.getRightX(),
        () -> false,
        () -> false));

        rightJoystick.button(2)
                .whileTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));
        controller.a().whileTrue(new InstantCommand(() ->
            scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.INTAKE)));

        controller.b().whileTrue(new InstantCommand(() ->
            scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.AIM)));
        
        controller.x().whileTrue(new InstantCommand(() ->
            scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.SHOOT)));

        controller.y().whileTrue(new InstantCommand(() ->
            scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.ABORT)));

        controller.back().whileTrue(new InstantCommand(() ->
            scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.ENDGAME)));
    }

    private void configureModes() {
    }

    public void configureSubsystems() {
        if (Constants.currentMode == Constants.Mode.SIM) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(driveTelemetry::telemeterize);

        switch (Constants.currentMode) {
            case REAL:
                scoringSubsystem = new ScoringSubsystem(new ShooterIOSim(), new AimerIOSim());
                break;
            case SIM:
                scoringSubsystem = new ScoringSubsystem(new ShooterIOSim(), new AimerIOSim());
                break;
            case REPLAY:
        }
    }
}
