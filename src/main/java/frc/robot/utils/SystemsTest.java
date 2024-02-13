// TODO: WIP - Not tested

package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

public class SystemsTest extends Command {
    private Tunable subsystem;
    private boolean inverted;

    private int slot;

    private Timer timer;

    public SystemsTest(Tunable subsystem) {
        this(subsystem, false, 0);
    }

    public SystemsTest(Tunable subsystem, boolean inverted) {
        this(subsystem, inverted, 0);
    }

    public SystemsTest(Tunable subsystem, boolean inverted, int slot) {
        this.subsystem = subsystem;
        this.inverted = inverted;
        this.slot = slot;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        subsystem.setVolts(inverted ? 0.5 : -0.5, slot);
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
            Logger.recordOutput("Test-Mode/" + subsystem.toString(), false);
            return true;
        } else if (Math.abs(subsystem.getVelocity(slot)) > 0.1) {
            return true;
        }
        return false;
    }
}
