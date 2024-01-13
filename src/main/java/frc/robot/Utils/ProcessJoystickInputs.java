package frc.robot.Utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;

public class ProcessJoystickInputs {
    

    public static double[] process(double x, double y, double rot, boolean babyMode) {
         double[] joystickInputsFiltered = Deadband.twoAxisDeadband(x, y,
                DriveConstants.deadbandPercent);

        double xMpS = joystickInputsFiltered[0] * DriveConstants.MaxSpeedMetPerSec;
        double yMpS = joystickInputsFiltered[1] * DriveConstants.MaxSpeedMetPerSec;
        double rotRadpS = Deadband.oneAxisDeadband(rot, DriveConstants.deadbandPercent)
                * DriveConstants.MaxAngularRateRadiansPerSec;

        if (babyMode) {
            xMpS *= 0.5;
            yMpS *= 0.5;
            rotRadpS *= 0.5;
        }

        double control[] = {-xMpS, -yMpS, -rotRadpS};
        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("z", rot);
        return control;
    }
}
