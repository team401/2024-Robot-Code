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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.AlignState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.AlignTarget;
import frc.robot.subsystems.endgame.EndgameIO;
import frc.robot.subsystems.endgame.EndgameIOSim;
import frc.robot.subsystems.endgame.EndgameIOSparkFlex;
import frc.robot.subsystems.endgame.EndgameSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeAction;
import frc.robot.subsystems.localization.CameraContainerReal;
import frc.robot.subsystems.localization.CameraContainerReplay;
import frc.robot.subsystems.localization.CameraContainerSim;
import frc.robot.subsystems.localization.VisionLocalizer;
import frc.robot.subsystems.scoring.AimerIO;
import frc.robot.subsystems.scoring.AimerIORoboRio;
import frc.robot.subsystems.scoring.AimerIOSim;
import frc.robot.subsystems.scoring.HoodIO;
import frc.robot.subsystems.scoring.HoodIOSim;
import frc.robot.subsystems.scoring.HoodIOSparkFlex;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringAction;
import frc.robot.subsystems.scoring.ShooterIO;
import frc.robot.subsystems.scoring.ShooterIOSim;
import frc.robot.subsystems.scoring.ShooterIOTalon;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryIO;
import frc.robot.telemetry.TelemetryIOLive;
import frc.robot.telemetry.TelemetryIOSim;
import frc.robot.utils.FieldFinder;
import frc.robot.utils.feedforward.TuneG;
import frc.robot.utils.feedforward.TuneS;
import frc.robot.utils.feedforward.TuneV;
import frc.robot.utils.ControllerJSONReader;
import frc.robot.utils.FieldFinder;
import java.util.Collections;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;
    EndgameSubsystem endgameSubsystem;

    HashMap<String, Trigger> triggers;
    HashMap<String, DoubleSupplier> axes;
    HashMap<String, IntSupplier> pov;

    VisionLocalizer tagVision;

    CommandSwerveDrivetrain drivetrain = FeatureFlags.runDrive ? TunerConstants.DriveTrain : null;
    Telemetry driveTelemetry;

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    SendableChooser<String> testModeChooser = new SendableChooser<String>();

    LED leds;

    public RobotContainer() {
        configureSubsystems();
        configureModes();
        configureAutonomous();
    }

    // spotless:off
    private void configureBindings() {
        ControllerJSONReader.pullConfiguration("SingleController");
        triggers = ControllerJSONReader.getTriggers();
        axes = ControllerJSONReader.getAxes();
        pov = ControllerJSONReader.getPOVs();

        if (FeatureFlags.runDrive) {
            drivetrain.setDefaultCommand(
                    new DriveWithJoysticks(
                            drivetrain,
                            axes.get("xDrive"),
                            axes.get("yDrive"),
                            axes.get("rotation"),
                            pov.get("pov"),
                            () -> true,
                            () -> false,
                            triggers.get("leftBumper"))); //unsure if using trigger as booleansupplier will work
                
            triggers.get("align")
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignState(AlignState.ALIGNING)))
                .onFalse(new InstantCommand(
                    () -> drivetrain.setAlignState(AlignState.MANUAL)));
        }

        if (FeatureFlags.runScoring) {
            triggers.get("intake")
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.INTAKE)))
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignState(AlignState.MANUAL)))
                .onTrue(new InstantCommand(
                    () -> intakeSubsystem.toggle()));

            triggers.get("aimSpeaker")
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                         ScoringSubsystem.ScoringAction.AIM)))
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.SPEAKER)));

            triggers.get("shoot")
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.SHOOT)))
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.SPEAKER)))
                .onFalse(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.AIM)));

            triggers.get("endgame")
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.ENDGAME)))
                .onFalse(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.WAIT)));

            triggers.get("aimAmp")
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.AMP_AIM)))
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.AMP)));

            triggers.get("manualIdle")
                .onTrue(new InstantCommand(
                    () -> scoringSubsystem.setAction(
                        ScoringSubsystem.ScoringAction.WAIT)))
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignState(AlignState.MANUAL)));
        }
    } // spotless:on

    private void configureModes() {}

    public void configureSubsystems() {
        switch (Constants.currentMode) {
            case REAL:
                if (FeatureFlags.runDrive) {
                    driveTelemetry =
                            new Telemetry(DriveConstants.MaxSpeedMetPerSec, new TelemetryIOLive());
                }

                if (FeatureFlags.runScoring) {
                    scoringSubsystem =
                            new ScoringSubsystem(
                                    new ShooterIOTalon(),
                                    new AimerIORoboRio(),
                                    new HoodIOSparkFlex());
                }

                if (FeatureFlags.runEndgame) {
                    endgameSubsystem = new EndgameSubsystem(new EndgameIOSparkFlex());
                }

                if (FeatureFlags.runIntake) {
                    intakeSubsystem = new IntakeSubsystem(new IntakeIOSparkMax());
                }

                if (FeatureFlags.runVision) {
                    tagVision =
                            new VisionLocalizer(new CameraContainerReal(VisionConstants.cameras));
                }
                break;
            case SIM:
                if (FeatureFlags.runDrive) {
                    driveTelemetry =
                            new Telemetry(DriveConstants.MaxSpeedMetPerSec, new TelemetryIOSim());

                    drivetrain.seedFieldRelative(DriveConstants.initialPose);
                }

                if (FeatureFlags.runEndgame) {
                    endgameSubsystem = new EndgameSubsystem(new EndgameIOSim());
                }

                if (FeatureFlags.runScoring) {
                    scoringSubsystem =
                            new ScoringSubsystem(
                                    new ShooterIOSim(), new AimerIOSim(), new HoodIOSim());
                }

                if (FeatureFlags.runVision) {
                    tagVision =
                            new VisionLocalizer(
                                    new CameraContainerSim(
                                            VisionConstants.cameras,
                                            driveTelemetry::getModuleStates));
                }

                if (FeatureFlags.runIntake) {
                    intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
                }
                break;
            case REPLAY:
                if (FeatureFlags.runDrive) {
                    driveTelemetry =
                            new Telemetry(DriveConstants.MaxSpeedMetPerSec, new TelemetryIO() {});

                    drivetrain.seedFieldRelative(DriveConstants.initialPose);
                }

                if (FeatureFlags.runEndgame) {
                    endgameSubsystem = new EndgameSubsystem(new EndgameIO() {});
                }

                if (FeatureFlags.runScoring) {
                    scoringSubsystem =
                            new ScoringSubsystem(
                                    new ShooterIO() {}, new AimerIO() {}, new HoodIO() {});
                }

                if (FeatureFlags.runIntake) {
                    intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
                }

                if (FeatureFlags.runVision) {
                    tagVision =
                            new VisionLocalizer(
                                    new CameraContainerReplay(VisionConstants.cameras.size()));
                }
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

        if (FeatureFlags.runScoring) {
            if (FeatureFlags.runDrive) {
                scoringSubsystem.setPoseSupplier(driveTelemetry::getFieldToRobot);

                scoringSubsystem.setVelocitySupplier(
                        () ->
                                VecBuilder.fill(
                                        driveTelemetry.getVelocityX(),
                                        driveTelemetry.getVelocityY()));

                scoringSubsystem.setSpeakerSupplier(this::getFieldToSpeaker);

                scoringSubsystem.setDriveAllignedSupplier(() -> drivetrain.isAligned());
            }
            if (FeatureFlags.runEndgame) {
                scoringSubsystem.setElevatorPositionSupplier(endgameSubsystem::getPosition);
            }
            if (FeatureFlags.runIntake) {
                intakeSubsystem.setScoringSupplier(scoringSubsystem::canIntake);
            }
        }

        if (FeatureFlags.enableLEDS) leds = new LED(scoringSubsystem);
    }

    // spotless:off
    private void configureBindings() {
        // Resets bindings
        controller = new CommandXboxController(2);
        
        if (FeatureFlags.runDrive) {
            drivetrain.setDefaultCommand(
                    new DriveWithJoysticks(
                            drivetrain,
                            () -> controller.getLeftY(),
                            () -> controller.getLeftX(),
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
                    () -> drivetrain.seedFieldRelative(getPoseAgainstSpeaker()))
                );

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

        if (FeatureFlags.runEndgame) {
            endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.OVERRIDE);

            controller.povUp()
                .onTrue(new InstantCommand(() -> endgameSubsystem.setVolts(2.0, 0)))
                .onFalse(new InstantCommand(() -> endgameSubsystem.setVolts(0.0, 0)));

            controller.povDown()
                .onTrue(new InstantCommand(() -> endgameSubsystem.setVolts(-2.0, 0)))
                .onFalse(new InstantCommand(() -> endgameSubsystem.setVolts(0.0, 0)));
        }

        if (FeatureFlags.runIntake) {
            controller.a()
                .onTrue(new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.INTAKE)));
                
            controller.start()
                .onTrue(new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.NONE)));
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

    private void configureModes() {
        testModeChooser.setDefaultOption("Blank", "tuning");

        testModeChooser.addOption("Speaker Tuning", "calculate-speaker");

        testModeChooser.addOption("Intake Tuning", "tuning-intake");
        testModeChooser.addOption("Aimer Tuning", "tuning-aimer");
        testModeChooser.addOption("Hood Tuning", "tuning-hood");
        testModeChooser.addOption("Shooter Tuning", "tuning-shooter");
        testModeChooser.addOption("Endgame Tuning", "tuning-endgame");

        SmartDashboard.putData("Test Mode Chooser", testModeChooser);
    }

    public void enabledInit() {
        if (FeatureFlags.runIntake) {
            intakeSubsystem.run(IntakeAction.NONE);
        }

        if (FeatureFlags.runEndgame) {
            endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.CANCEL);
        }
    }

    public void testInit() {
        // Resets bindings
        controller = new CommandXboxController(2);

        // spotless:off
        switch (testModeChooser.getSelected()) {
            case "tuning":
                break;
            case "calculate-speaker":
                drivetrain.seedFieldRelative();
                scoringSubsystem.setAction(ScoringAction.TUNING);
                controller.leftBumper()
                        .onTrue(new InstantCommand(
                            () -> scoringSubsystem.setTuningKickerVolts(5)))
                        .onFalse(new InstantCommand(
                            () -> scoringSubsystem.setTuningKickerVolts(0)));
                break;
            case "tuning-intake":
                if (FeatureFlags.runDrive) {
                    drivetrain.setDefaultCommand(
                        new DriveWithJoysticks(
                            drivetrain,
                            () -> controller.getLeftY(),
                            () -> controller.getLeftX(),
                            () -> -controller.getRightX(),
                            () -> -1,
                            () -> true,
                            () -> false,
                            () -> false));
                }

                SmartDashboard.putNumber("Test-Mode/intake/intakeVolts", 1.0);
                SmartDashboard.putNumber("Test-Mode/intake/beltVolts", 1.0);

                intakeSubsystem.run(IntakeSubsystem.IntakeAction.OVERRIDE);

                controller.leftBumper()
                    .onTrue(new InstantCommand(() -> intakeSubsystem.setOverrideVolts(
                        SmartDashboard.getNumber("Test-Mode/intake/intakeVolts", 1.0),
                        SmartDashboard.getNumber("Test-Mode/intake/beltVolts", 1.0))))
                    .onFalse(new InstantCommand(() -> intakeSubsystem.setOverrideVolts(0, 0)));

                controller.rightBumper()
                    .onTrue(new InstantCommand(() -> intakeSubsystem.setOverrideVolts(
                        -SmartDashboard.getNumber("Test-Mode/intake/intakeVolts", 1.0),
                        -SmartDashboard.getNumber("Test-Mode/intake/beltVolts", 1.0))))
                    .onFalse(new InstantCommand(() -> intakeSubsystem.setOverrideVolts(0, 0)));
                break;
            case "tuning-aimer":
                SmartDashboard.putNumber("Test-Mode/aimer/kP", ScoringConstants.aimerkP);
                SmartDashboard.putNumber("Test-Mode/aimer/kI", ScoringConstants.aimerkI);
                SmartDashboard.putNumber("Test-Mode/aimer/kD", ScoringConstants.aimerkD);

                SmartDashboard.putNumber("Test-Mode/aimer/profileMaxVelocity", ScoringConstants.aimCruiseVelocity);
                SmartDashboard.putNumber("Test-Mode/aimer/profileMaxAcceleration", ScoringConstants.aimAcceleration);

                SmartDashboard.putNumber("Test-Mode/aimer/setpointPosition", 0.25);
                SmartDashboard.putNumber("Test-Mode/aimer/volts", 2.0);

                scoringSubsystem.setAction(ScoringAction.OVERRIDE);

                controller.a()
                    .onTrue(new TuneS(scoringSubsystem, 0));

                controller.b()
                    .onTrue(new TuneG(scoringSubsystem, 0));

                controller.y()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setPID(
                        SmartDashboard.getNumber("Test-Mode/aimer/kP", ScoringConstants.aimerkP),
                        SmartDashboard.getNumber("Test-Mode/aimer/kI", ScoringConstants.aimerkI),
                        SmartDashboard.getNumber("Test-Mode/aimer/kD", ScoringConstants.aimerkD), 0)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setMaxProfileProperties(
                        SmartDashboard.getNumber("Test-Mode/aimer/profileMaxVelocity", ScoringConstants.aimCruiseVelocity),
                        SmartDashboard.getNumber("Test-Mode/aimer/profileMaxAcceleration", ScoringConstants.aimAcceleration), 0)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(SmartDashboard.getNumber("Test-Mode/aimer/setpointPosition", 0.25), 0)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.TEMPORARY_SETPOINT)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                controller.povUp()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(1.1, 0)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.TEMPORARY_SETPOINT)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));
                
                controller.povDown()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(0.0, 0)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.TEMPORARY_SETPOINT)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                controller.leftBumper()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(SmartDashboard.getNumber("Test-Mode/aimer/volts", 2.0), 0)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 0)));

                controller.rightBumper()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(-SmartDashboard.getNumber("Test-Mode/aimer/volts", 2.0), 0)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 0)));
                break;
            case "tuning-hood":
                SmartDashboard.putNumber("Test-Mode/hood/kP", ScoringConstants.hoodkP);
                SmartDashboard.putNumber("Test-Mode/hood/kI", ScoringConstants.hoodkI);
                SmartDashboard.putNumber("Test-Mode/hood/kD", ScoringConstants.hoodkD);

                SmartDashboard.putNumber("Test-Mode/hood/setpointPosition", 0.75);

                scoringSubsystem.setAction(ScoringAction.OVERRIDE);

                controller.x().onTrue(new InstantCommand(() -> scoringSubsystem.homeHood()));

                controller.y()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setPID(
                        SmartDashboard.getNumber("Test-Mode/hood/kP", ScoringConstants.hoodkP),
                        SmartDashboard.getNumber("Test-Mode/hood/kI", ScoringConstants.hoodkI),
                        SmartDashboard.getNumber("Test-Mode/hood/kD", ScoringConstants.hoodkD), 1)))
                    .onTrue(new InstantCommand(() ->scoringSubsystem.runToPosition(SmartDashboard.getNumber("Test-Mode/hood/setpointPosition", 0.75), 1)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.TEMPORARY_SETPOINT)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                controller.povDown()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(0.0, 1)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.TEMPORARY_SETPOINT)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                controller.leftBumper()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(1.0, 1)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 1)));

                controller.rightBumper()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(-1.0, 1)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 1)));
                break;
            case "tuning-shooter":
                SmartDashboard.putNumber("Test-Mode/shooter/kP", ScoringConstants.shooterkP);
                SmartDashboard.putNumber("Test-Mode/shooter/kI", ScoringConstants.shooterkI);
                SmartDashboard.putNumber("Test-Mode/shooter/kD", ScoringConstants.shooterkD);

                SmartDashboard.putNumber("Test-Mode/shooter/kS", ScoringConstants.shooterkS);
                SmartDashboard.putNumber("Test-Mode/shooter/kV", ScoringConstants.shooterkV);
                SmartDashboard.putNumber("Test-Mode/shooter/kA", ScoringConstants.shooterkA);

                SmartDashboard.putNumber("Test-Mode/shooter/setpointRPM", 2000);

                scoringSubsystem.setAction(ScoringAction.OVERRIDE);

                controller.y()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setPID(
                        SmartDashboard.getNumber("Test-Mode/shooter/kP", ScoringConstants.shooterkP),
                        SmartDashboard.getNumber("Test-Mode/shooter/kI", ScoringConstants.shooterkI),
                        SmartDashboard.getNumber("Test-Mode/shooter/kD", ScoringConstants.shooterkD), 2)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setFF(
                        SmartDashboard.getNumber("Test-Mode/shooter/kS", ScoringConstants.shooterkS),
                        SmartDashboard.getNumber("Test-Mode/shooter/kV", ScoringConstants.shooterkV),
                        SmartDashboard.getNumber("Test-Mode/shooter/kA", ScoringConstants.shooterkA),
                        0.0, 2)))
                    .onTrue(new InstantCommand(() ->scoringSubsystem.runToPosition(SmartDashboard.getNumber("Test-Mode/shooter/setpointRPM", 2000), 2)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.TEMPORARY_SETPOINT)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                controller.povLeft().onTrue(new InstantCommand(() -> scoringSubsystem.setTuningKickerVolts(2.0))).onFalse(new InstantCommand(() -> scoringSubsystem.setTuningKickerVolts(0.0)));
                controller.povRight().onTrue(new InstantCommand(() -> scoringSubsystem.setTuningKickerVolts(-2.0))).onFalse(new InstantCommand(() -> scoringSubsystem.setTuningKickerVolts(0.0)));

                controller.povDown()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(0.0, 2)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.TEMPORARY_SETPOINT)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));
                
                controller.leftBumper()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(10.0, 2)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 2)));

                controller.rightBumper()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(-10.0, 2)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 2)));
                break;
            case "tuning-endgame":
                // TODO: Add in later
                // SmartDashboard.putNumber("Test-Mode/endgame/kP", EndgameConstants.kP);
                // SmartDashboard.putNumber("Test-Mode/endgame/kI", EndgameConstants.kI);
                // SmartDashboard.putNumber("Test-Mode/endgame/kD", EndgameConstants.kD);

                SmartDashboard.putNumber("Test-Mode/endgame/volts", 1.0);

                endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.OVERRIDE);

                controller.a()
                    .onTrue(new TuneS(endgameSubsystem, 0));

                controller.b()
                    .onTrue(new TuneG(endgameSubsystem, 0));

                controller.x()
                    .onTrue(new TuneV(endgameSubsystem, 1.0, 0));
                
                controller.leftBumper()
                    .onTrue(new InstantCommand(() -> endgameSubsystem.setVolts(SmartDashboard.getNumber("Test-Mode/endgame/volts", 1.0), 0)))
                    .onFalse(new InstantCommand(() -> endgameSubsystem.setVolts(0, 0)));

                controller.rightBumper()
                    .onTrue(new InstantCommand(() -> endgameSubsystem.setVolts(-SmartDashboard.getNumber("Test-Mode/endgame/volts", 1.0), 0)))
                    .onFalse(new InstantCommand(() -> endgameSubsystem.setVolts(0, 0)));
                break;
        }
        // spotless:on
    }

    public void testPeriodic() {}

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

    private Pose2d getPoseAgainstSpeaker() {
        if (DriverStation.getAlliance().isEmpty()) {
            return FieldConstants.robotAgainstRedSpeaker;
        } else {
            switch (DriverStation.getAlliance().get()) {
                case Blue:
                    return FieldConstants.robotAgainstBlueSpeaker;
                case Red:
                    return FieldConstants.robotAgainstRedSpeaker;
            }
        }
        throw new RuntimeException("Unreachable branch of switch expression");
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

    public void teleopInit() {
        configureBindings();

        // This is in teleopInit to prevent it from wasting time in auto
        if (FeatureFlags.runScoring) {
            scoringSubsystem.homeHood();

            scoringSubsystem.setAction(ScoringAction.WAIT);
        }
    }
}
