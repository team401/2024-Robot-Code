// TODO: WIP - Not tested

package frc.robot.utils.feedforward;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Tunable;
import java.util.ArrayList;

public class TuneV extends Command {
    private Tunable subsystem;

    private double volts;

    private ArrayList<Double> velocities;

    double startPosition; // TODO

    double kS;
    double pastkV;
    double average = 0;
    double vel = 0;

    public TuneV(Tunable subsystem, double volts) {
        this.subsystem = subsystem;
        this.volts = volts;
        this.kS = SmartDashboard.getNumber("kS", 0);
        this.pastkV = SmartDashboard.getNumber("kV", 0);

        // this.withTimeout(5); TODO: Maybe add?
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Ended", false);
        subsystem.setVolts(volts);
        velocities = new ArrayList<Double>();
    }

    @Override
    public void execute() {
        vel = subsystem.getVel();
        SmartDashboard.putNumber("Velocity", vel);
        if (Math.abs(subsystem.getPosition()) < 0.6) {
            velocities.add(vel);
        }
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Ended", true);
        subsystem.stop();

        for (double v : velocities) {
            average += v;
        }

        average /= velocities.size();

        SmartDashboard.putNumber("kV", ((volts - kS) / average) + pastkV);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(subsystem.getPosition()) > 1;
    }
}
