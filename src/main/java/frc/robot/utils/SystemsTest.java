// TODO: WIP - Not tested

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class SystemsTest extends Command {
    private Tunable subsystem;
    private boolean inverted;

    private Timer timer;

    public SystemsTest(Tunable subsystem) {
        this(subsystem, false);
    }

    public SystemsTest(Tunable subsystem, boolean inverted) {
        this.subsystem = subsystem;
        this.inverted = inverted;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        subsystem.setVolts(inverted ? 0.5 : -0.5);
        timer.start();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(0.5)) {
            System.out.println(
                    "*********************************** Faliure with "
                            + subsystem.toString()
                            + " ***********************************");
            return true;
        } else if (subsystem.getVel() > 0.1) {
            return true;
        }
        return false;
    }
}
