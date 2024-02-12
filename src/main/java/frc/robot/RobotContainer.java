package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.AlignState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.AlignTarget;
import frc.robot.subsystems.endgame.EndgameIOSim;
import frc.robot.subsystems.endgame.EndgameIOSparkFlex;
import frc.robot.subsystems.endgame.EndgameSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeAction;
import frc.robot.subsystems.localization.VisionIOReal;
import frc.robot.subsystems.localization.VisionIOSim;
import frc.robot.subsystems.localization.VisionLocalizer;
import frc.robot.subsystems.scoring.AimerIOSim;
import frc.robot.subsystems.scoring.AimerIOTalon;
import frc.robot.subsystems.scoring.HoodIOSim;
import frc.robot.subsystems.scoring.HoodIOSparkFlex;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import frc.robot.subsystems.scoring.ShooterIOSim;
import frc.robot.subsystems.scoring.ShooterIOTalon;
import frc.robot.utils.FieldFinder;
import java.util.Collections;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;
    EndgameSubsystem endgameSubsystem;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController controller = new CommandXboxController(2);

    VisionLocalizer tagVision;

    CommandSwerveDrivetrain drivetrain = FeatureFlags.runDrive ? TunerConstants.DriveTrain : null;
    Telemetry driveTelemetry = new Telemetry(DriveConstants.MaxSpeedMetPerSec);

    SendableChooser<String> autoChooser = new SendableChooser<String>();

    public RobotContainer() {
        configureSubsystems();
        configureBindings();
        configureModes();
        configureAutonomous();
    }

    // spotless:off
    private void configureBindings() {
        if (FeatureFlags.runDrive) {
            drivetrain.setDefaultCommand(
                    new DriveWithJoysticks(
                            drivetrain,
                            () -> -controller.getLeftY(),
                            () -> -controller.getLeftX(),
                            () -> -controller.getRightX(),
                            () -> controller.getHID().getPOV(),
                            () -> true,
                            () -> false,
                            () -> controller.getHID().getLeftBumper()));
                
            controller.rightBumper()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignState(AlignState.ALIGNING)))
                .onFalse(new InstantCommand(
                    () -> drivetrain.setAlignState(AlignState.MANUAL)));

            controller.leftBumper()
                .onTrue(new InstantCommand(
                    () -> drivetrain.seedFieldRelative(new Pose2d(1, 1.5, new Rotation2d()))
                ));

            controller.b()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.SPEAKER)));

            controller.x()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.SPEAKER)));

            controller.back()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.AMP)));
        }

        if (FeatureFlags.runIntake) {
            controller.a()
                .onTrue(new InstantCommand(
                        () -> intakeSubsystem.toggle()));
        }

        if (FeatureFlags.runScoring) {
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
                        ScoringSubsystem.ScoringAction.AMP_AIM)));

            controller.start()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.WAIT)));
        }
    } // spotless:on

    private void configureModes() {}

    public void configureSubsystems() {
        switch (Constants.currentMode) {
            case REAL:
                if (FeatureFlags.runScoring) {
                    if (FeatureFlags.runEndgame) {
                        endgameSubsystem = new EndgameSubsystem(new EndgameIOSparkFlex());
                    }

                    scoringSubsystem =
                            new ScoringSubsystem(
                                    new ShooterIOTalon(),
                                    new AimerIOTalon(),
                                    new HoodIOSparkFlex(),
                                    driveTelemetry::getFieldToRobot,
                                    () ->
                                            VecBuilder.fill(
                                                    driveTelemetry.getVelocityX(),
                                                    driveTelemetry.getVelocityY()),
                                    this::getFieldToSpeaker,
                                    endgameSubsystem::getPosition);
                }

                if (FeatureFlags.runIntake) {
                    intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax());
                }

                if (FeatureFlags.runVision) {
                    tagVision = new VisionLocalizer(new VisionIOReal(VisionConstants.cameras));
                }
                break;
            case SIM:
                if (FeatureFlags.runDrive) {
                    drivetrain.seedFieldRelative(DriveConstants.initialPose);
                }

                if (FeatureFlags.runEndgame) {
                    endgameSubsystem = new EndgameSubsystem(new EndgameIOSim());
                }

                if (FeatureFlags.runScoring) {
                    scoringSubsystem =
                            new ScoringSubsystem(
                                    new ShooterIOSim(),
                                    new AimerIOSim(),
                                    new HoodIOSim(),
                                    driveTelemetry::getFieldToRobot,
                                    () ->
                                            VecBuilder.fill(
                                                    driveTelemetry.getVelocityX(),
                                                    driveTelemetry.getVelocityY()),
                                    this::getFieldToSpeaker,
                                    endgameSubsystem::getPosition);
                }

                if (FeatureFlags.simulateVision) {
                    tagVision =
                            new VisionLocalizer(
                                    new VisionIOSim(
                                            VisionConstants.cameras,
                                            driveTelemetry::getModuleStates));
                } else if (FeatureFlags.runVision) {
                    tagVision =
                            new VisionLocalizer(
                                    new VisionIOSim(
                                            Collections.emptyList(),
                                            driveTelemetry::getModuleStates));
                }

                if (FeatureFlags.runIntake) {
                    intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
                }
                break;
            case REPLAY:
                break;
        }

        if (FeatureFlags.runDrive) {
            drivetrain.registerTelemetry(driveTelemetry::telemeterize);
            drivetrain.setPoseSupplier(driveTelemetry::getFieldToRobot);
            drivetrain.setVelocitySupplier(driveTelemetry::getVelocity);
            drivetrain.setSpeakerSupplier(this::getFieldToSpeaker);
            drivetrain.setAmpSupplier(this::getFieldToAmpHeading);
            drivetrain.setSourceSupplier(this::getFieldToSourceHeading);

            if (FeatureFlags.runVision) {
                tagVision.setCameraConsumer(
                        (m) ->
                                drivetrain.addVisionMeasurement(
                                        m.pose(), m.timestamp(), m.variance()));
                tagVision.setFieldToRobotSupplier(driveTelemetry::getFieldToRobot);
            }
        }

        if (FeatureFlags.runScoring) {}

        if (FeatureFlags.runIntake && FeatureFlags.runScoring) {
            intakeSubsystem.setScoringSupplier(scoringSubsystem::canIntake);
        }
    }

    public void enabledInit() {
        if (FeatureFlags.runScoring) {
            scoringSubsystem.setAction(ScoringAction.WAIT);
        }
    }

    public void testInit(String choice) {
        switch (choice) {
            case "tuning":
                break;
            case "tuning-speaker":
                drivetrain.seedFieldRelative();
                scoringSubsystem.setAction(ScoringAction.TUNING);
                // spotless:off
                controller.leftBumper()
                        .onTrue(new InstantCommand(
                            () -> scoringSubsystem.setTuningKickerVolts(5)))
                        .onFalse(new InstantCommand(
                            () -> scoringSubsystem.setTuningKickerVolts(0)));
                break;
                // spotless:on
        }
    }

    private Translation2d getFieldToSpeaker() {
        if (DriverStation.getAlliance().isEmpty()) {
            return FieldConstants.fieldToBlueSpeaker;
        } else {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    Logger.recordOutput("Field/speaker", FieldConstants.fieldToBlueSpeaker);
                    return FieldConstants.fieldToBlueSpeaker;
                case Red:
                    Logger.recordOutput("Field/speaker", FieldConstants.fieldToRedSpeaker);
                    return FieldConstants.fieldToRedSpeaker;
            }
        }
        throw new RuntimeException("Unreachable branch of switch expression");
    }

    private Rotation2d getFieldToAmpHeading() {
        Logger.recordOutput("Field/amp", FieldConstants.fieldToAmpHeading);
        return FieldConstants.fieldToAmpHeading;
    }

    private Rotation2d getFieldToSourceHeading() {
        if (DriverStation.getAlliance().isEmpty()) {
            return FieldConstants.fieldToRedSourceHeading;
        } else {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    Logger.recordOutput("Field/source", FieldConstants.fieldToBlueSourceHeading);
                    return FieldConstants.fieldToBlueSourceHeading;
                case Red:
                    Logger.recordOutput("Field/source", FieldConstants.fieldToRedSourceHeading);
                    return FieldConstants.fieldToRedSourceHeading;
            }
        }
        throw new RuntimeException("Unreachable branch of switch expression");
    }

    public void robotPeriodic() {
        if (FeatureFlags.runDrive) {
            Logger.recordOutput(
                    "localizer/whereAmI",
                    FieldFinder.whereAmI(
                            driveTelemetry.getFieldToRobot().getTranslation().getX(),
                            driveTelemetry.getFieldToRobot().getTranslation().getY()));

            Logger.recordOutput("localizer/RobotPose", driveTelemetry.getFieldToRobot());
            Logger.recordOutput(
                    "localizer/RobotVelocity",
                    new Pose2d(
                            driveTelemetry.getFieldToRobot().getX()
                                    + (driveTelemetry.getVelocity().getX()
                                            * DriveConstants.anticipationTime),
                            driveTelemetry.getFieldToRobot().getY()
                                    + (driveTelemetry.getVelocity().getY()
                                            * DriveConstants.anticipationTime),
                            driveTelemetry.getFieldToRobot().getRotation()));

            driveTelemetry.logDataSynchronously();
        }
    }

    public Command getAutonomousCommand() {
        return drivetrain.getAutoPath(autoChooser.getSelected());
    }

    private void configureAutonomous() {
        autoChooser.setDefaultOption("Default (S3 6-Note)", "S3-W3-W2-W1-C1-C2");

        autoChooser.addOption("S1 5-Note", "S1-W1-W2-W3-C5");
        autoChooser.addOption("S1 4-Note", "S1-W1-W2-W3");

        autoChooser.addOption("S2 4-Note", "S1-W1-W2-W3");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand(
                "Shoot Scoring",
                new InstantCommand(
                        () -> {
                            scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.SHOOT);
                            intakeSubsystem.run(IntakeAction.INTAKE);
                        }));
        NamedCommands.registerCommand(
                "Aim Scoring",
                new InstantCommand(
                        () -> scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.AIM)));
        NamedCommands.registerCommand(
                "Intake Scoring",
                new InstantCommand(
                        () -> scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.INTAKE)));
        NamedCommands.registerCommand(
                "Wait Scoring",
                new InstantCommand(
                        () -> scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.WAIT)));
    }
}
