// TODO: WIP - Not tested

package frc.robot.Utils.FeedForward;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.Tunable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuneS extends Command {
    private Tunable subsystem;

    double startPosition;

    double appliedVolts;

    public TuneS(Tunable subsystem) {
        this.subsystem = subsystem;

        // this.withTimeout(5); TODO: Maybe add?
    }

    @Override
    public void initialize() {
        startPosition = subsystem.getPosition();
        appliedVolts = 0;
    }

    @Override
    public void execute() {
        subsystem.setVolts(appliedVolts);
        appliedVolts += 0.001;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
        SmartDashboard.putNumber("kS", appliedVolts);
    }

    @Override
    public boolean isFinished() {
        return subsystem.getVel() > 0.1;
    }
}
