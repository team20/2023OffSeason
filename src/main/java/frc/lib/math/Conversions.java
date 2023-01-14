package frc.lib.math;

public class Conversions {

    /**
     * @param positionCounts CANCoder Position Counts
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between CANCoder and Mechanism
     * @return CANCoder Position Counts
     */
    public static double degreesToCANcoder(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param counts Falcon Position Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double sparkToDegrees(double positionCounts, double gearRatio) {
        return positionCounts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Position Counts
     */
    public static double degreesToSpark(double degrees, double gearRatio) {
        return degrees / (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts Spark Rotation per Second
     * @param gearRatio Gear Ratio between NEO and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double sparkToRPM(double velocityCounts, double gearRatio) {
        double mechRPM = velocityCounts / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between NEO and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Motor
     */
    public static double RPMToSpark(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        return motorRPM;
    }

    /**
     * @param velocitycounts Spark RPM
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between NEO and Mechanism (set to 1 for Falcon MPS)
     * @return Meters per Second
     */
    public static double sparkToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = sparkToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between NEO and Mechanism (set to 1 for Falcon MPS)
     * @return Spark Velocity Counts
     */
    public static double MPSToSpark(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToSpark(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param positionCounts Spark Position Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between NEO and Wheel
     * @return Meters
     */
    public static double sparkToMeters(double positionCounts, double circumference, double gearRatio){
        return positionCounts * circumference;
    }

    /**
     * @param meters Meters
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between NEO and Wheel
     * @return Spark Position Counts
     */
    public static double MetersToSpark(double meters, double circumference, double gearRatio){
        return meters / circumference;
    }
}