// TODO: WIP - Not tested

package frc.robot.utils.feedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Tunable;

public class TuneG extends Command {
    private Tunable subsystem;

    double startPosition;

    double kG;
    double kS;

    public TuneG(Tunable subsystem) {
        this.subsystem = subsystem;
        this.kS = SmartDashboard.getNumber("kS", 0);

        // this.withTimeout(5); TODO: Maybe add?
    }

    @Override
    public void initialize() {
        startPosition = subsystem.getPosition();
        kG = kS;
    }

    @Override
    public void execute() {
        subsystem.setVolts(-kG);
        kG += 0.001;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
        SmartDashboard.putNumber("kG", kG - kS);
    }

    @Override
    public boolean isFinished() {
        return subsystem.getPosition() > Math.abs(startPosition - 0.1);
    }
}
