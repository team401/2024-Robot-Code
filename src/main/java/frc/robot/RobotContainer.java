package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
=======
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.drive.DriveWithJoysticks;
import frc.robot.drive.Telemetry;
import frc.robot.Constants.TunerConstants;
import frc.robot.drive.CommandSwerveDrivetrain;
>>>>>>> 23a2431 (rest of honzik's code copied in, errors removed, advantagekit removed somewhat)

public class RobotContainer {
    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController controller = new CommandXboxController(2);
<<<<<<< HEAD
=======
    CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

    Telemetry driveTelemetry = new Telemetry(DriveConstants.MaxSpeedMetPerSec);

    private void configureBindings() {
        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain,
                () -> controller.getLeftX(),
                () -> controller.getLeftY(),
                () -> controller.getRightX(),
                () -> rightJoystick.trigger().getAsBoolean(),
                () -> leftJoystick.trigger().getAsBoolean()));

        rightJoystick.button(2)
                .whileTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        drivetrain.registerTelemetry(driveTelemetry::telemeterize);
        SmartDashboard.putBoolean("on", true);
    }

    public void configureSubsystems() {
    }
>>>>>>> 23a2431 (rest of honzik's code copied in, errors removed, advantagekit removed somewhat)

    public RobotContainer() {
        configureBindings();
        configureSubsystems();
<<<<<<< HEAD
        configureModes();
    }

    private void configureBindings() {}

    private void configureSubsystems() {}

    private void configureModes() {}
}
=======
    }

    public void disabledPeriodic() {
    }

    public void enabledInit() {
    }
}
>>>>>>> 23a2431 (rest of honzik's code copied in, errors removed, advantagekit removed somewhat)
