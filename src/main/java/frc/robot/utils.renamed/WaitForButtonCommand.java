package frc.robot.Utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class WaitForButtonCommand extends Command {
    private CommandXboxController controller;

    private Timer time = new Timer();

    public WaitForButtonCommand(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return controller.a().getAsBoolean() && time.get() > 0.5;
    }
}
