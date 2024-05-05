package frc.robot.utils.monitors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

public class MonitoredSubsystem extends SubsystemBase {
    private List<Monitor> registeredMonitors = new ArrayList<Monitor>();

    public void addMonitor(Monitor monitor) {
        registeredMonitors.add(monitor);
    }

    @Override
    public void periodic() {
        monitoredPeriodic();
        runMonitors();
    }

    /**
     * OVERRIDE ME! This function is called every time the subsystem's periodic function is called.
     * However, MonitoredSubsytem automatically checks monitors during every periodic run.
     * Therefore, this method should be overridden as a replacement for the normal periodic function
     * in the implementation of the subsystem.
     *
     * <p>This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should
     * try to be consistent within their own codebases about which responsibilities will be handled
     * by Commands, and which will be handled here.
     */
    public void monitoredPeriodic() {}

    private void runMonitors() {
        registeredMonitors.forEach(
                monitor -> {
                    monitor.periodic();
                });
    }
}
