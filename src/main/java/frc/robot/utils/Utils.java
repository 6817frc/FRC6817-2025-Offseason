package frc.robot.utils;

public class Utils {
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
