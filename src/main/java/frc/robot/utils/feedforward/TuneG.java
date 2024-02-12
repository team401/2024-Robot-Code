// TODO: WIP - Not tested

package frc.robot.utils.feedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Tunable;

public class TuneG extends Command {
    private Tunable subsystem;

    private int slot;

    double startPosition;

    double kG;
    double kS;

    public TuneG(Tunable subsystem, int slot) {
        this.subsystem = subsystem;
        this.kS = SmartDashboard.getNumber("kS", 0);
        this.slot = slot;

        // this.withTimeout(5); TODO: Maybe add?
    }

    @Override
    public void initialize() {
        startPosition = subsystem.getPosition(slot);
        kG = kS;
    }

    @Override
    public void execute() {
        subsystem.setVolts(-kG, slot);
        kG += 0.001;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setVolts(0.0, slot);
        SmartDashboard.putNumber("kG", kG - kS);
    }

    @Override
    public boolean isFinished() {
        return subsystem.getPosition(slot) > Math.abs(startPosition - 0.1);
    }
}
