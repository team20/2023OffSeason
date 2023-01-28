package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

public class CalibrationAutoCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;

    public static enum Operation {
        CMD_ANGLE, CMD_DISTANCE
    }

    Operation m_op;
    double m_amount = 0; // if distance, in ticks; if angle, in degrees
  /***
   * Autonomous command to steer and drive
   * 
   * @param op
   *                        Enumerated mode-steer (CMD_ANGLE) or drive (CMD_DISTANCE)
   * @param amount
   *                        op == CMD_ANGLE: amount is angle in degrees,
   *                        op == CMD_DISTANCE: amount is distance in meters
   * @see Operation
   */
    public CalibrationAutoCommand(Operation op, double amount) { 
        m_driveSubsystem = DriveSubsystem.get();
        m_op = op;
        if (m_op == Operation.CMD_DISTANCE) {
            double wheelCirc = Math.PI * Constants.SwerveConstants.wheelDiameter;
            double metersToTicks = (Constants.SwerveConstants.ticksPerAxisRev * Constants.SwerveConstants.gearRatio * 39.37) / wheelCirc;
            m_amount = amount * metersToTicks; 
        } else if (m_op == Operation.CMD_ANGLE) { 
            m_amount = amount;
        } else {
            try {
                throw new Exception("Unsupported command");
            } catch (Exception e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        addRequirements(DriveSubsystem.get());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (m_op == Operation.CMD_ANGLE) { 
            m_driveSubsystem.setSteerMotors(m_amount, m_amount, m_amount, m_amount);
        } else {
            // reset drive encoders
            m_driveSubsystem.m_frontLeftDriveEncoder.setPosition(0);
            m_driveSubsystem.m_frontRightDriveEncoder.setPosition(0);
            m_driveSubsystem.m_backLeftDriveEncoder.setPosition(0);
            m_driveSubsystem.m_backRightDriveEncoder.setPosition(0);

            // turn on motors - set 10% power for now
            m_driveSubsystem.setDriveMotors(.1, .1, .1, .1);
        }
    }

    @Override
    public boolean isFinished() {
        switch(m_op){
            case CMD_ANGLE:
                //The error between the actual angle and the target angle
                double error = m_driveSubsystem.getFrontLeftCANCoder().getPosition() - Math.toRadians(m_amount);
                return Math.abs(error) < Math.toRadians((1));
            case CMD_DISTANCE:
                //Determine whether the target distance has been reached
                return (m_driveSubsystem.m_frontLeftDriveEncoder.getPosition() >= m_amount);
        }
                
        return false;
    }

    @Override
    public void cancel() {
        m_driveSubsystem.setDriveMotors(0, 0, 0, 0);
    }
}