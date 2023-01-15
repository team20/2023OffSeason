package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class CTREModuleState {

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially reversing the direction the wheel spins. Customized from WPILib's version to
     * include placing in appropriate range for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        
        // Figure smallest delta in angle from the current angle to the desired angle.
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double delta = targetAngle - currentAngle.getDegrees();

        // Instead of turning the entire wheel around, reverse speed and adjust angle.
        double targetSpeed = desiredState.speedMetersPerSecond;
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * Returns the closest angle in degrees within the range [0.0, 360.0).
     * 
     * @param currentAngleDeg 
     *   Current angle, degrees.
     * @param newAngleDeg
     *   Target angle, degrees.
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double currentAngleDeg, double newAngleDeg) {

        // TODO get some sample values and add a test case
        
        double lowerBound;
        double upperBound;

        double lowerOffset = currentAngleDeg % 360;
        if (lowerOffset >= 0) {
            lowerBound = currentAngleDeg - lowerOffset;
            upperBound = currentAngleDeg + (360 - lowerOffset);
        } else {
            upperBound = currentAngleDeg - lowerOffset;
            lowerBound = currentAngleDeg - (360 + lowerOffset);
        }

        while (newAngleDeg < lowerBound) {
            newAngleDeg += 360;
        }
        while (newAngleDeg > upperBound) {
            newAngleDeg -= 360;
        }

        if (newAngleDeg - currentAngleDeg > 180) {
            newAngleDeg -= 360;
        } else if (newAngleDeg - currentAngleDeg < -180) {
            newAngleDeg += 360;
        }

        return newAngleDeg;
    }
}
