// TODO: WIP - Not tested

package frc.robot.utils.feedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Tunable;

public class TuneS extends Command {
    private Tunable subsystem;

    private int slot;

    double startPosition;

    double appliedVolts;

    public TuneS(Tunable subsystem, int slot) {
        this.subsystem = subsystem;
        this.slot = slot;

        // this.withTimeout(5); TODO: Maybe add?
    }

    @Override
    public void initialize() {
        startPosition = subsystem.getPosition(slot);
        appliedVolts = 0;
    }

    @Override
    public void execute() {
        subsystem.setVolts(appliedVolts, slot);
        appliedVolts += 0.001;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setVolts(0.0, slot);
        SmartDashboard.putNumber("kS", appliedVolts);
    }

    @Override
    public boolean isFinished() {
        return subsystem.getVelocity(slot) > 0.1;
    }
}
