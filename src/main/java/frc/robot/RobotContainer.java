package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.endgame.EndgameSimIO;
import frc.robot.subsystems.endgame.EndgameSparkMaxIO;
import frc.robot.subsystems.endgame.EndgameSubsystem;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.VisionIOReal;
import frc.robot.subsystems.localization.VisionIOSim;
import frc.robot.subsystems.localization.VisionLocalizer;
import frc.robot.subsystems.scoring.AimerIOSim;
import frc.robot.subsystems.scoring.AimerIOTalon;
import frc.robot.subsystems.scoring.HoodIOSim;
import frc.robot.subsystems.scoring.HoodIOVortex;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import frc.robot.subsystems.scoring.ShooterIOSim;
import frc.robot.subsystems.scoring.ShooterIOTalon;
import frc.robot.utils.FieldFinder;
import java.util.Collections;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    ScoringSubsystem scoringSubsystem;

    IntakeSubsystem intake;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController controller = new CommandXboxController(2);

    VisionLocalizer tagVision;

    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    Telemetry driveTelemetry = new Telemetry(DriveConstants.MaxSpeedMetPerSec);

    EndgameSubsystem endgameSubsystem;

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
                        () -> controller.getHID().getPOV(),
                        () -> true,
                        () -> false,
                        () -> controller.getHID().getRightBumper()));

        controller.a()
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.INTAKE)))
                .onTrue(new InstantCommand(
                    () -> intake.toggle()
                ));

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
                                                driveTelemetry.getVelocityY()),
                                this::getFieldToSpeaker);

                tagVision = new VisionLocalizer(new VisionIOReal(VisionConstants.cameras));
                endgameSubsystem = new EndgameSubsystem(new EndgameSparkMaxIO());
                break;
            case SIM:
                drivetrain.seedFieldRelative(DriveConstants.initialPose);

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
                                this::getFieldToSpeaker);

                if (FeatureFlags.simulateVision) {
                    tagVision =
                            new VisionLocalizer(
                                    new VisionIOSim(
                                            VisionConstants.cameras,
                                            driveTelemetry::getModuleStates));
                } else {
                    tagVision =
                            new VisionLocalizer(
                                    new VisionIOSim(
                                            Collections.emptyList(),
                                            driveTelemetry::getModuleStates));
                }
                endgameSubsystem = new EndgameSubsystem(new EndgameSimIO());

                intake = new IntakeSubsystem(new IntakeIOSim());
                break;
            case REPLAY:
                break;
        }

        drivetrain.registerTelemetry(driveTelemetry::telemeterize);
        drivetrain.setPoseSupplier(driveTelemetry::getFieldToRobot);
        drivetrain.setSpeakerSupplier(this::getFieldToSpeaker);
        drivetrain.setAmpSupplier(this::getFieldToAmpHeading);
        drivetrain.setSourceSupplier(this::getFieldToSourceHeading);

        intake.setScoringSupplier(scoringSubsystem::canIntake);

        tagVision.setCameraConsumer(
                (m) -> drivetrain.addVisionMeasurement(m.pose(), m.timestamp(), m.variance()));
        tagVision.setFieldToRobotSupplier(driveTelemetry::getFieldToRobot);
    }

    public void enabledInit() {
        scoringSubsystem.setAction(ScoringAction.WAIT);
    }

    public void testInit(String choice) {
        switch (choice) {
            case "tuning":
                break;
            case "tuning-speaker":
                drivetrain.seedFieldRelative();
                scoringSubsystem.setAction(ScoringAction.TUNING);
                // spotless:off
                controller.rightBumper()
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
            return FieldConstants.fieldToRedSpeaker;
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
        Logger.recordOutput(
                "localizer/whereAmI",
                FieldFinder.whereAmI(
                        driveTelemetry.getFieldToRobot().getTranslation().getX(),
                        driveTelemetry.getFieldToRobot().getTranslation().getY()));

        driveTelemetry.logDataSynchronously();
    }
}
