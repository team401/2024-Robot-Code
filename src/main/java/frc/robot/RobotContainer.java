package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    CommandJoystick leftJoystick = new CommandJoystick(0);
    CommandJoystick rightJoystick = new CommandJoystick(1);
    CommandXboxController controller = new CommandXboxController(2);

    public RobotContainer() {
        configureBindings();
        configureSubsystems();
        configureModes();
    }

    private void configureBindings() {}

    private void configureSubsystems() {}

    private void configureModes() {}
}
