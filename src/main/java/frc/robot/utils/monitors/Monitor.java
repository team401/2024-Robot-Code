package frc.robot.utils.monitors;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Monitor {
    String name; // Name to log the status of the monitor under
    boolean sticky; // Should the monitor still report a fault after conditions return to normal?
    double timeToFault; // How long the value can be unacceptable before a fault occurs
    BooleanSupplier isStateValid; // Supplier with which to check whether the value is acceptable
    Runnable faultCallback; // Function to call when the fault happens

    Timer triggeredTime;

    boolean triggered = false; // Is the value currently unnacceptable?
    boolean faulted = false; // Has the monitor detected a fault?

    public Monitor(
            String name,
            boolean sticky,
            BooleanSupplier isStateValid,
            double timeToFault,
            Runnable faultCallback) {
        this.name = name;
        this.sticky = sticky;
        this.timeToFault = timeToFault;
        this.isStateValid = isStateValid;

        this.faultCallback = faultCallback;

        triggeredTime = new Timer();
    }

    public void periodic() {
        triggered = !isStateValid.getAsBoolean();
        if (triggered) {
            triggeredTime.start();

            if (triggeredTime.get() > timeToFault) {
                faulted = true;
            }
        } else {
            if (!sticky) {
                faulted = false;
            }

            triggeredTime.reset();
            triggeredTime.stop();
        }
        if (faulted) {
            faultCallback.run();
        }

        Logger.recordOutput("monitors/" + name + "/triggered", triggered);
        Logger.recordOutput("monitors/" + name + "/faulted", faulted);
    }

    public boolean isFaulted() {
        return faulted;
    }

    public boolean isTriggered() {
        return triggered;
    }
}
