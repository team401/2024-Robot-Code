package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ConversionConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndgameConstants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.ShootWithGamepad;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.AlignState;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.AlignTarget;
import frc.robot.subsystems.endgame.EndgameIO;
import frc.robot.subsystems.endgame.EndgameIOSim;
import frc.robot.subsystems.endgame.EndgameIOSparkFlex;
import frc.robot.subsystems.endgame.EndgameSubsystem;
import frc.robot.subsystems.endgame.EndgameSubsystem.EndgameAction;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeAction;
import frc.robot.subsystems.localization.CameraContainerReal;
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
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.FieldFinder;
import frc.robot.utils.feedforward.TuneG;
import frc.robot.utils.feedforward.TuneS;
import frc.robot.utils.notesimulator.Note;
import frc.robot.utils.notesimulator.NoteManager;
import lib.coppercore.wpi_interface.ControllerJSONReader;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    ScoringSubsystem scoringSubsystem;
    IntakeSubsystem intakeSubsystem;
    EndgameSubsystem endgameSubsystem;

    ControllerJSONReader controllerJSONReader;

    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController controller = new CommandXboxController(2);

    VisionLocalizer tagVision;

    CommandSwerveDrivetrain drivetrain = FeatureFlags.runDrive ? TunerConstants.DriveTrain : null;
    Telemetry driveTelemetry;

    SendableChooser<String> testModeChooser = new SendableChooser<String>();

    DigitalInput brakeSwitch = new DigitalInput(IOConstants.brakeSwitchPort);
    DigitalInput ledSwitch = new DigitalInput(IOConstants.ledSwitchPort);

    DigitalOutput timeDigitalOutput = null; // new DigitalOutput(IOConstants.timeOutputPort);

    LED leds;

    Command endgameCommand = Commands.print("Endgame command not constructed");

    public RobotContainer() {
        configureSubsystems();
        configureModes();
        configureAutonomous();
        if (Constants.currentMode == Mode.SIM) {
            NoteManager.addNote(new Note(driveTelemetry::getFieldToRobot, true, "1"));
            NoteManager.addNote(
                    new Note(
                            driveTelemetry::getFieldToRobot,
                            new Pose2d(2.69, 4.14, new Rotation2d()),
                            "2"));
            NoteManager.addNote(
                    new Note(
                            driveTelemetry::getFieldToRobot,
                            "" + (NoteManager.numberOfExistingNotes() + 1)));
            NoteManager.addNote(
                    new Note(
                            driveTelemetry::getFieldToRobot,
                            "" + (NoteManager.numberOfExistingNotes() + 1)));
            NoteManager.addNote(
                    new Note(
                            driveTelemetry::getFieldToRobot,
                            "" + (NoteManager.numberOfExistingNotes() + 1)));
            NoteManager.addNote(
                    new Note(
                            driveTelemetry::getFieldToRobot,
                            "" + (NoteManager.numberOfExistingNotes() + 1)));
        }
        if (FeatureFlags.runDrive) {
            drivetrain.configurePathPlanner();
        }

        SmartDashboard.putNumber("Debug/currentTimeMillis", System.currentTimeMillis());
    }

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
                            new VisionLocalizer(new CameraContainerReplay(VisionConstants.cameras));
                }
                break;
        }

        if (FeatureFlags.runDrive) {
            drivetrain.registerTelemetry(driveTelemetry::telemeterize);
            drivetrain.setPoseSupplier(driveTelemetry::getFieldToRobot);
            drivetrain.setVelocitySupplier(driveTelemetry::getVelocity);
        }

        if (FeatureFlags.runScoring) {
            if (FeatureFlags.runDrive) {
                scoringSubsystem.setPoseSupplier(driveTelemetry::getFieldToRobot);

                scoringSubsystem.setVelocitySupplier(
                        () ->
                                VecBuilder.fill(
                                        driveTelemetry.getVelocityX(),
                                        driveTelemetry.getVelocityY()));

                scoringSubsystem.setDriveAllignedSupplier(() -> drivetrain.isAligned());
            }
            if (FeatureFlags.runEndgame) {
                scoringSubsystem.setElevatorPositionSupplier(endgameSubsystem::getPosition);

                if (FeatureFlags.runDrive) {
                    endgameCommand =
                            new EndgameSequence(
                                    scoringSubsystem,
                                    endgameSubsystem,
                                    drivetrain,
                                    driveTelemetry,
                                    () -> controller.getHID().getLeftBumper());
                }
            }
            if (FeatureFlags.runIntake) {
                intakeSubsystem.setScoringSupplier(() -> !scoringSubsystem.hasNote());
            }
        }

        if (FeatureFlags.runVision) {
            tagVision.setCameraConsumer(
                    (m) -> drivetrain.addVisionMeasurement(m.pose(), m.timestamp(), m.variance()));
        }

        if (FeatureFlags.enableLEDS) {
            leds = new LED(scoringSubsystem, intakeSubsystem);

            if (FeatureFlags.runVision) {
                leds.setVisionWorkingSupplier(() -> tagVision.coprocessorConnected());
            }
        }
    }

    // spotless:off
    private void configureBindings() {
        // Resets bindings
        controller = new CommandXboxController(2);

        if (FeatureFlags.runDrive) {
            setUpDriveWithJoysticks();
                
            leftJoystick.trigger()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignState(AlignState.ALIGNING)))
                .onFalse(new InstantCommand(
                    () -> drivetrain.setAlignState(AlignState.MANUAL)));

            leftJoystick.top()
                .onTrue(new InstantCommand(
                    () -> drivetrain.seedFieldRelative(AllianceUtil.getPoseAgainstSpeaker())));

            leftJoystick.button(3)
                .onTrue(new InstantCommand(
                    () -> drivetrain.seedFieldRelative(AllianceUtil.getPoseAgainstSpeakerLeft())));

            leftJoystick.button(4)
                .onTrue(new InstantCommand(
                    () -> drivetrain.seedFieldRelative(AllianceUtil.getPoseAgainstSpeakerRight())));
            
            leftJoystick.povUp()
                .onTrue(new InstantCommand(
                    () -> drivetrain.seedFieldRelative(AllianceUtil.getPoseAgainstPodium())));

            controller.povUp()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.SPEAKER)));

            controller.povRight()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.AMP)));

            controller.povLeft()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.SOURCE)));

            controller.povDown()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.ENDGAME)));
            
            rightJoystick.povUp()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.UP)));

            rightJoystick.povDown()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.DOWN)));
          
            rightJoystick.povLeft()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.LEFT)));

            rightJoystick.povRight()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignTarget(AlignTarget.RIGHT)));

            controller.x()
                .onTrue(new InstantCommand(() -> drivetrain.driveToSource()))
                .onFalse(new InstantCommand(() -> drivetrain.stopDriveToPose()));

            controller.y()
                .onTrue(new InstantCommand(() -> drivetrain.driveToSpeaker()))
                .onFalse(new InstantCommand(() -> drivetrain.stopDriveToPose()));
        }

        if (FeatureFlags.runEndgame) {
            endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.WAIT);

            controller.leftBumper()
                .onTrue(new InstantCommand(() -> {
                    endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_UP);
                    scoringSubsystem.forceHood(true);
                }));

            controller.leftTrigger()
                .onTrue(new InstantCommand(() -> {
                    endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.GO_DOWN);
                    scoringSubsystem.forceHood(false);
                }));
            
            // controller.x()
            //     .onTrue(new InstantCommand(() -> {
            //         endgameSubsystem.setAction(EndgameAction.OVERRIDE);
            //         endgameSubsystem.setVolts(-3, 0);
            //         scoringSubsystem.forceHood(false);
            //     })).onFalse(new InstantCommand(() -> {
            //         endgameSubsystem.setVolts(0, 0);
            //     }));
        }

        if (FeatureFlags.runIntake) {
            controller.b()
                .onTrue(new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.INTAKE)))
                .onTrue(
                    new InstantCommand(() -> NoteManager.intake()))
                .onFalse(new InstantCommand(
                    () -> intakeSubsystem.run(IntakeAction.NONE)));

            controller.a()
                .onTrue(new InstantCommand(
                    () -> intakeSubsystem.run(IntakeAction.REVERSE)))
                .onFalse(new InstantCommand(
                    () -> intakeSubsystem.run(IntakeAction.NONE)));

            // HACK: This button was added during DCMP to un-jam the intake. Ideally, this functionality should be implemented through a state machine.
            controller.x()
                .onTrue(new SequentialCommandGroup(new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.REVERSE)),
                    Commands.waitSeconds(0.1),
                    new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.INTAKE)),
                    Commands.waitSeconds(0.5),
                    new InstantCommand(
                        () -> intakeSubsystem.run(IntakeAction.NONE))))
                .onFalse(new InstantCommand(
                    () -> intakeSubsystem.run(IntakeAction.NONE)));
        }

        if (FeatureFlags.runScoring) {
            scoringSubsystem.setDefaultCommand(new ShootWithGamepad(
                () -> rightJoystick.getHID().getRawButton(4),
                controller.getHID()::getRightBumper,
                controller.getHID()::getYButton,
                () -> controller.getRightTriggerAxis() > 0.5,
                controller.getHID()::getAButton,
                controller.getHID()::getBButton, scoringSubsystem,
                FeatureFlags.runDrive ? drivetrain::getAlignTarget : () -> AlignTarget.NONE));

            rightJoystick.button(11).onTrue(new InstantCommand(() -> scoringSubsystem.setArmDisabled(true)));
            rightJoystick.button(16).onTrue(new InstantCommand(() -> scoringSubsystem.setArmDisabled(false)));

            rightJoystick.button(12).onTrue(new InstantCommand(
                () -> {
                    scoringSubsystem.setAction(ScoringAction.OVERRIDE);
                    scoringSubsystem.setVolts(3, 0);
                }, scoringSubsystem));

            rightJoystick.button(15).onTrue(new InstantCommand(
                () -> {
                    scoringSubsystem.setAction(ScoringAction.OVERRIDE);
                    scoringSubsystem.setVolts(-3, 0);
                }, scoringSubsystem));
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
        testModeChooser.addOption("Wheel Characterization", "characterization-wheel");
        testModeChooser.addOption("Translation Drive Tuning", "drive-translation-tuning");
        testModeChooser.addOption("Rotation Drive Tuning", "drive-rotation-tuning");
        testModeChooser.addOption("Drive Align Tuning", "drive-align");
        testModeChooser.addOption("Drive Velocity Tuning", "drive-velocity");

        SmartDashboard.putData("Test Mode Chooser", testModeChooser);
    }

    public void enabledInit() {
        if (timeDigitalOutput != null) {
            timeDigitalOutput.set(true);
        }

        if (FeatureFlags.runIntake) {
            intakeSubsystem.run(IntakeAction.NONE);
        }

        if (FeatureFlags.runScoring) {
            scoringSubsystem.enabledInit();
            scoringSubsystem.setBrakeMode(true);
        }

        if (FeatureFlags.runEndgame) {
            endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.WAIT);
            endgameSubsystem.setBrakeMode(true);
        }

        if (FeatureFlags.enableLEDS) {
            leds.setEnabled(true);
        }
    }

    public void disabledInit() {
        if (timeDigitalOutput != null) {
            timeDigitalOutput.set(false);
        }

        if (FeatureFlags.runEndgame) {
            endgameSubsystem.setAction(EndgameAction.WAIT);
        }

        if (FeatureFlags.runDrive) {
            // HACK: this method is called here to stop the drive from briefly moving when the robot
            // is enabled. Either the method should be renamed, or the underlying issue
            // (unreasonable
            // disabledPeriodic() loop times) should be rectified and this call moved.
            drivetrain.teleopInit();
        }
    }

    public void testInit() {
        // Resets bindings
        controller = new CommandXboxController(2);

        // spotless:off
        switch (testModeChooser.getSelected()) {
            case "tuning":
                break;
            case "drive-align":
                SmartDashboard.putNumber("Test-Mode/drive/alignPMax", Constants.DriveConstants.alignmentkPMax);
                SmartDashboard.putNumber("Test-Mode/drive/alignPMin", Constants.DriveConstants.alignmentkPMin);
                SmartDashboard.putNumber("Test-Mode/drive/alignI", Constants.DriveConstants.alignmentkI);
                SmartDashboard.putNumber("Test-Mode/drive/alignD", Constants.DriveConstants.alignmentkD);
            
                setUpDriveWithJoysticks();

                controller.a()
                    .onTrue(new InstantCommand(
                        () -> drivetrain.setAlignTarget(AlignTarget.SPEAKER)));

                controller.b()
                    .onTrue(new InstantCommand(
                        () -> drivetrain.setAlignState(AlignState.ALIGNING)))
                    .onFalse(new InstantCommand(
                        () -> drivetrain.setAlignState(AlignState.MANUAL)));

                controller.b()
                .onTrue(new InstantCommand(
                    () -> drivetrain.setAlignGains(
                        SmartDashboard.getNumber("Test-Mode/drive/alignPMax", Constants.DriveConstants.alignmentkPMax),
                        SmartDashboard.getNumber("Test-Mode/drive/alignPMin", Constants.DriveConstants.alignmentkPMin),
                        SmartDashboard.getNumber("Test-Mode/drive/alignI", Constants.DriveConstants.alignmentkI),
                        SmartDashboard.getNumber("Test-Mode/drive/alignD", Constants.DriveConstants.alignmentkD)
                    )));
                break;
            case "calculate-speaker":
                drivetrain.seedFieldRelative();

                setUpDriveWithJoysticks();
                scoringSubsystem.setAction(ScoringAction.TUNING);

                SmartDashboard.putNumber("Test-Mode/aimer/setpointPosition", 1.0);
                SmartDashboard.putNumber("Test-Mode/shooter/setpointRPM", 2000);

                controller.y()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(SmartDashboard.getNumber("Test-Mode/aimer/setpointPosition", 1.0), 0)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.runToPosition(SmartDashboard.getNumber("Test-Mode/shooter/setpointRPM", 2000), 2)))
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.TEMPORARY_SETPOINT)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setAction(ScoringAction.OVERRIDE)));

                controller.leftBumper()
                        .onTrue(new InstantCommand(
                            () -> scoringSubsystem.setTuningKickerVolts(10)))
                        .onFalse(new InstantCommand(
                                () -> scoringSubsystem.setTuningKickerVolts(0)));
                break;
            case "tuning-intake":
                if (FeatureFlags.runDrive) {
                    setUpDriveWithJoysticks();
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

                SmartDashboard.putNumber("Test-Mode/hood/maxVelocity", ScoringConstants.hoodMaxVelocity);
                SmartDashboard.putNumber("Test-Mode/hood/maxAcceleration", ScoringConstants.hoodMaxAcceleration);

                SmartDashboard.putNumber("Test-Mode/hood/setpointPosition", 0.75);
                SmartDashboard.putNumber("Test-Mode/hood/volts", 1.0);

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
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(SmartDashboard.getNumber("Test-Mode/hood/volts", 1.0), 1)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 1)));

                controller.rightBumper()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(-SmartDashboard.getNumber("Test-Mode/hood/volts", 1.0), 1)))
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
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(12.0, 2)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 2)));

                controller.rightBumper()
                    .onTrue(new InstantCommand(() -> scoringSubsystem.setVolts(-12.0, 2)))
                    .onFalse(new InstantCommand(() -> scoringSubsystem.setVolts(0, 2)));
                break;
            case "tuning-endgame":
                SmartDashboard.putNumber("Test-Mode/endgame/kP", EndgameConstants.climberkP);
                SmartDashboard.putNumber("Test-Mode/endgame/kI", EndgameConstants.climberkI);
                SmartDashboard.putNumber("Test-Mode/endgame/kD", EndgameConstants.climberkD);

                SmartDashboard.putNumber("Test-Mode/endgame/kFF", EndgameConstants.climberkFFClimber);

                SmartDashboard.putNumber("Test-Mode/endgame/setpoint", EndgameConstants.climberTargetUpMeters);

                SmartDashboard.putNumber("Test-Mode/endgame/maxVelocity", EndgameConstants.climberProfileConstraints.maxVelocity);
                SmartDashboard.putNumber("Test-Mode/endgame/maxAcceleration", EndgameConstants.climberProfileConstraints.maxAcceleration);

                SmartDashboard.putNumber("Test-Mode/endgame/volts", 1.0);

                endgameSubsystem.setAction(EndgameSubsystem.EndgameAction.OVERRIDE);

                controller.povUp()
                        .onTrue(new InstantCommand(() -> endgameSubsystem
                                .setVolts(SmartDashboard.getNumber("Test-Mode/endgame/volts", 1.0), 0)))
                        .onFalse(new InstantCommand(() -> endgameSubsystem.setVolts(0, 0)));

                controller.povDown()
                        .onTrue(new InstantCommand(() -> endgameSubsystem
                                .setVolts(-SmartDashboard.getNumber("Test-Mode/endgame/volts", 1.0), 0)))
                        .onFalse(new InstantCommand(() -> endgameSubsystem.setVolts(0, 0)));

                controller.a()
                        .onTrue(new InstantCommand(() -> endgameSubsystem
                                .setVolts(SmartDashboard.getNumber("Test-Mode/endgame/volts", 0.0), 0)))
                        .onFalse(
                                new InstantCommand(
                                        () -> {
                                            endgameSubsystem.setVolts(0, 0);
                                            endgameSubsystem.setAction(EndgameAction.OVERRIDE);
                                        }));

                controller.b()
                        .onTrue(new TuneG(endgameSubsystem, 0));

                controller.y()
                        .onTrue(new InstantCommand(() -> endgameSubsystem.setPID(
                                SmartDashboard.getNumber("Test-Mode/endgame/kP", 0.0),
                                SmartDashboard.getNumber("Test-Mode/endgame/kI", 0.0),
                                SmartDashboard.getNumber("Test-Mode/endgame/kD", 0.0), 0)))
                        .onTrue(new InstantCommand(() -> endgameSubsystem.setFF(
                                0, 0, 0, SmartDashboard.getNumber("Test-Mode/endgame/kFF", 0), 0)))
                        .onTrue(new InstantCommand(() -> endgameSubsystem.setAction(EndgameAction.TEMPORARY_SETPOINT)))
                        .onTrue(new InstantCommand(() -> endgameSubsystem.setMaxProfileProperties(SmartDashboard.getNumber("Test-Mode/endgame/maxVelocity", EndgameConstants.climberProfileConstraints.maxVelocity), SmartDashboard.getNumber("Test-Mode/endgame/maxAcceleration", EndgameConstants.climberProfileConstraints.maxAcceleration), 0)))
                        .onTrue(new InstantCommand(() -> endgameSubsystem
                                .runToPosition(SmartDashboard.getNumber("Test-Mode/endgame/setpoint", 0), 0)))
                        .onFalse(
                                new InstantCommand(
                                        () -> {
                                            endgameSubsystem.setVolts(0, 0);
                                            endgameSubsystem.setAction(EndgameAction.OVERRIDE);
                                        }));

                controller.x()
                        .onTrue(new InstantCommand(() -> endgameSubsystem.setAction(EndgameAction.TEMPORARY_SETPOINT)))
                        .onTrue(new InstantCommand(() -> endgameSubsystem
                                .runToPosition(0.0, 0)))
                        .onFalse(
                                new InstantCommand(
                                        () -> {
                                            endgameSubsystem.setVolts(0, 0);
                                            endgameSubsystem.setAction(EndgameAction.OVERRIDE);
                                        }));

                controller.leftBumper()
                        .onTrue(new InstantCommand(() -> endgameSubsystem.setPID(
                                SmartDashboard.getNumber("Test-Mode/endgame/kP", 0.0),
                                SmartDashboard.getNumber("Test-Mode/endgame/kI", 0.0),
                                SmartDashboard.getNumber("Test-Mode/endgame/kD", 0.0), 0)))
                        .onTrue(new InstantCommand(() -> endgameSubsystem.setAction(EndgameAction.GO_DOWN)))
                        .onFalse(
                                new InstantCommand(
                                        () -> {
                                            endgameSubsystem.setVolts(0, 0);
                                            endgameSubsystem.setAction(EndgameAction.OVERRIDE);
                                        }));


                controller.rightBumper()
                        .onTrue(new InstantCommand(() -> endgameSubsystem.setPID(
                                SmartDashboard.getNumber("Test-Mode/endgame/kP", 0.0),
                                SmartDashboard.getNumber("Test-Mode/endgame/kI", 0.0),
                                SmartDashboard.getNumber("Test-Mode/endgame/kD", 0.0), 0)))
                        .onTrue(new InstantCommand(() -> endgameSubsystem.setAction(EndgameAction.GO_UP)))
                        .onFalse(
                                new InstantCommand(
                                        () -> {
                                            endgameSubsystem.setVolts(0, 0);
                                            endgameSubsystem.setAction(EndgameAction.OVERRIDE);
                                        }));
                    break;
            case "characterization-wheel":
                controller.a()
                        .whileTrue(
                                new WheelRadiusCharacterization(
                                        drivetrain,
                                        () -> drivetrain.getPigeon2().getYaw().getValueAsDouble()
                                                * ConversionConstants.kDegreesToRadians));
                break;
            case "drive-translation-tuning":
                SmartDashboard.putNumber("Test-Mode/drive/translationVolts", 0.0);

                drivetrain.setAlignState(AlignState.SYS_ID_DRIVE);

                controller.a()
                        .onTrue(new InstantCommand(() ->
                            drivetrain.setTuningVolts(SmartDashboard.getNumber("Test-Mode/drive/translationVolts", 0.0))))
                        .onFalse(new InstantCommand(() ->
                            drivetrain.setTuningVolts(0.0)));
                break;
            case "drive-rotation-tuning":
                SmartDashboard.putNumber("Test-Mode/drive/rotationVolts", 0.0);

                drivetrain.setAlignState(AlignState.SYS_ID_ROTATION);

                controller.a()
                        .onTrue(new InstantCommand(() ->
                            drivetrain.setTuningVolts(SmartDashboard.getNumber("Test-Mode/drive/rotationVolts", 0.0))))
                        .onFalse(new InstantCommand(() ->
                            drivetrain.setTuningVolts(0.0)));
                break;
            case "drive-velocity":
                SmartDashboard.putNumber("Test-Mode/drive/velocity", 1.0);

                SmartDashboard.putNumber("Test-Mode/drive/velKP", TunerConstants.driveGains.kP);
                SmartDashboard.putNumber("Test-Mode/drive/velKI", TunerConstants.driveGains.kI);
                SmartDashboard.putNumber("Test-Mode/drive/velKD", TunerConstants.driveGains.kD);

                drivetrain.setAlignState(AlignState.MANUAL);

                controller.rightBumper()
                        .onTrue(new InstantCommand(() ->
                            drivetrain.setDrivePID(
                                SmartDashboard.getNumber("Test-Mode/drive/velKP", TunerConstants.driveGains.kP),
                                SmartDashboard.getNumber("Test-Mode/drive/velKI", TunerConstants.driveGains.kI),
                                SmartDashboard.getNumber("Test-Mode/drive/velKD", TunerConstants.driveGains.kD)
                            )));

                controller.a()
                        .onTrue(new InstantCommand(() -> {
                                drivetrain.setDrivePID(
                                        SmartDashboard.getNumber("Test-Mode/drive/velKP", TunerConstants.driveGains.kP),
                                        SmartDashboard.getNumber("Test-Mode/drive/velKI", TunerConstants.driveGains.kI),
                                        SmartDashboard.getNumber("Test-Mode/drive/velKD", TunerConstants.driveGains.kD)
                                );
                                drivetrain.setGoalChassisSpeeds(
                                    new ChassisSpeeds(
                                        SmartDashboard.getNumber("Test-Mode/drive/velocity", 1.0),
                                        0.0,
                                        0.0), 
                                    false);
                                
                            }))
                        .onFalse(new InstantCommand(() -> 
                            drivetrain.setGoalChassisSpeeds(
                                new ChassisSpeeds(0.0, 0.0, 0.0), 
                                false)));
                controller.b()
                        .onTrue(new InstantCommand(() -> {
                                drivetrain.setDrivePID(
                                        SmartDashboard.getNumber("Test-Mode/drive/velKP", TunerConstants.driveGains.kP),
                                        SmartDashboard.getNumber("Test-Mode/drive/velKI", TunerConstants.driveGains.kI),
                                        SmartDashboard.getNumber("Test-Mode/drive/velKD", TunerConstants.driveGains.kD)
                                );
                                drivetrain.setGoalChassisSpeeds(
                                    new ChassisSpeeds(
                                        -SmartDashboard.getNumber("Test-Mode/drive/velocity", 1.0),
                                        0.0,
                                        0.0), 
                                    false);
                                
                            }))
                        .onFalse(new InstantCommand(() -> 
                            drivetrain.setGoalChassisSpeeds(
                                new ChassisSpeeds(0.0, 0.0, 0.0), 
                                false)));
                break;
            }
        }
        // spotless:on

    public void testPeriodic() {}

    private void setUpDriveWithJoysticks() {
        if (FeatureFlags.runDrive) {
            drivetrain.setDefaultCommand(
                    new DriveWithJoysticks(
                            drivetrain,
                            () -> leftJoystick.getY(),
                            () -> leftJoystick.getX(),
                            () -> rightJoystick.getX(),
                            () -> !rightJoystick.trigger().getAsBoolean(),
                            () -> rightJoystick.top().getAsBoolean(),
                            () ->
                                    VecBuilder.fill(
                                            driveTelemetry.getVelocityX(),
                                            driveTelemetry.getVelocityY())));
        }
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

    public void disabledPeriodic() {
        /* set to coast mode when circuit open */
        if (brakeSwitch != null && brakeSwitch.get()) {
            if (FeatureFlags.runScoring) {
                scoringSubsystem.setBrakeMode(false);
            }
            if (FeatureFlags.runEndgame) {
                endgameSubsystem.setBrakeMode(false);
            }
            if (FeatureFlags.runDrive) {
                drivetrain.setBrakeMode(false);
            }
        } else {
            if (FeatureFlags.runScoring) {
                scoringSubsystem.setBrakeMode(true);
            }
            if (FeatureFlags.runEndgame) {
                endgameSubsystem.setBrakeMode(true);
            }
            if (FeatureFlags.runDrive) {
                drivetrain.setBrakeMode(true);
            }
        }
        if (ledSwitch != null && leds != null) {
            leds.setEnabled(!ledSwitch.get());
        }
    }

    public void autoInit() {
        if (drivetrain.getAutoCommand() != null) {
            drivetrain.autoInit();

            drivetrain.getAutoCommand().schedule();

            if (FeatureFlags.runScoring) {
                scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.SHOOT);
            }
            if (FeatureFlags.runIntake) {
                intakeSubsystem.run(IntakeAction.INTAKE);
            }
        }
    }

    private void configureAutonomous() {
        if (FeatureFlags.runScoring) {
            NamedCommands.registerCommand(
                    "Shoot Scoring",
                    new InstantCommand(
                            () -> {
                                if (FeatureFlags.runScoring) {
                                    scoringSubsystem.setAction(
                                            ScoringSubsystem.ScoringAction.SHOOT);
                                }
                                if (FeatureFlags.runIntake) {
                                    intakeSubsystem.run(IntakeAction.INTAKE);
                                }
                            }));
            NamedCommands.registerCommand(
                    "Aim Scoring",
                    new InstantCommand(
                            () -> scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.AIM)));
            NamedCommands.registerCommand(
                    "Intake Scoring",
                    new InstantCommand(
                            () ->
                                    scoringSubsystem.setAction(
                                            ScoringSubsystem.ScoringAction.INTAKE)));
            NamedCommands.registerCommand(
                    "Wait Scoring",
                    new InstantCommand(
                            () -> scoringSubsystem.setAction(ScoringSubsystem.ScoringAction.WAIT)));
            NamedCommands.registerCommand(
                    "OverrideStageAvoidance",
                    new InstantCommand(() -> scoringSubsystem.setOverrideStageAvoidance(true)));
            NamedCommands.registerCommand(
                    "Un-OverrideStageAvoidance",
                    new InstantCommand(() -> scoringSubsystem.setOverrideStageAvoidance(false)));
        } else {
            NamedCommands.registerCommand("Shoot Scoring", Commands.none());
            NamedCommands.registerCommand("Aim Scoring", Commands.none());
            NamedCommands.registerCommand("Wait Scoring", Commands.none());
            NamedCommands.registerCommand("Intake Scoring", Commands.none());
            NamedCommands.registerCommand("OverrideStageAvoidance", Commands.none());
            NamedCommands.registerCommand("Un-OverrideStageAvoidance", Commands.none());
        }
        if (FeatureFlags.runIntake) {
            NamedCommands.registerCommand("Intake Note", Commands.none());
        } else {
            NamedCommands.registerCommand("Intake Note", Commands.none());
        }
        if (FeatureFlags.runDrive) {
            NamedCommands.registerCommand(
                    "Disable Auto-Align",
                    new InstantCommand(() -> drivetrain.setAlignState(AlignState.MANUAL)));
            NamedCommands.registerCommand(
                    "Enable Auto-Align",
                    new InstantCommand(() -> drivetrain.setAlignState(AlignState.ALIGNING)));
        } else {
            NamedCommands.registerCommand("Disable Auto-Align", Commands.none());
            NamedCommands.registerCommand("Enable Auto-Align", Commands.none());
        }
        if (FeatureFlags.runScoring) {
            NamedCommands.registerCommand(
                    "Override Shoot",
                    new InstantCommand(() -> scoringSubsystem.setOverrideShoot(true)));
            NamedCommands.registerCommand(
                    "Un-Override Shoot",
                    new InstantCommand(() -> scoringSubsystem.setOverrideShoot(false)));
        } else {
            NamedCommands.registerCommand("Override Shoot", Commands.none());
            NamedCommands.registerCommand("Un-Override Shoot", Commands.none());
        }
    }

    public void teleopInit() {
        configureBindings();

        // This is in teleopInit to prevent it from wasting time in auto
        if (FeatureFlags.runScoring) {
            // scoringSubsystem.homeHood();

            scoringSubsystem.setAction(ScoringAction.WAIT);
            scoringSubsystem.enabledInit();
        }

        if (FeatureFlags.runDrive) {
            drivetrain.teleopInit();
        }

        SmartDashboard.putNumber("Debug/currentTimeMillis", System.currentTimeMillis());
    }
}
