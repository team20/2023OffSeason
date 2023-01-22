package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrationAutoCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;

    enum Operation {
        CMD_ANGLE, CMD_DISTANCE
    }

    Operation m_op;
    double m_amount = 0; // if distance, in ticks; if angle, in degrees
    public CalibrationAutoCommand(Operation op, double amount) {
        m_driveSubsystem = DriveSubsystem.get();
        m_op = op;
        if (m_op == Operation.CMD_DISTANCE) {
            m_amount = amount * 326.6369f;
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

            // turn on motors - set speed
            m_driveSubsystem.setDriveMotors(.1, .1, .1, .1);
        }
        // m_driveSubsystem.setSteerMotors(m_target, m_target, m_target, m_target);

        // todo: turn on motors
        // todo: reset distance encoders to zero
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_driveSubsystem.m_frontLeftDriveEncoder.getPosition()) >= m_amount) {
            return true;
        } 
        /*
         * double error = m_driveSubsystem.getFrontLeftSteerEncoderPosition() -
         * Math.toRadians(90);
         * if (Math.abs(error) < Math.toRadians(1)){
         * return true;
         * }
         */
                // check encoders to see if distance reached
        return false;
    }

    @Override
    public void cancel() {
        m_driveSubsystem.setDriveMotors(0, 0, 0, 0);
    }
}