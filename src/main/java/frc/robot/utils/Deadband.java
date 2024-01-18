package frc.robot.utils;

public class Deadband {
    // 1D Deadband in linear format
    public static double oneAxisDeadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0;
        } else {
            return input;
        }
    }

    // 2D Deadband in a circular format
    public static double[] twoAxisDeadband(double inputX, double inputY, double deadband) {
        double[] output = new double[2];
        if (Math.sqrt(Math.pow(inputX, 2) + Math.pow(inputY, 2)) < deadband) {
            output[0] = 0;
            output[1] = 0;
        } else {
            output[0] = inputX;
            output[1] = inputY;
        }
        return output;
    }

    // 3D Deadband in a spherical format
    public static double[] threeAxisDeadband(
            double inputX, double inputY, double inputZ, double deadband) {
        double[] output = new double[3];
        if (Math.sqrt(Math.pow(inputX, 2) + Math.pow(inputY, 2) + Math.pow(inputZ, 2)) < deadband) {
            output[0] = 0;
            output[1] = 0;
            output[2] = 0;
        } else {
            output[0] = inputX;
            output[1] = inputY;
            output[2] = inputZ;
        }
        return output;
    }
}
