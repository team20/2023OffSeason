package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CalibrationAutoCommand extends CommandBase {
    private final DriveSubsystem m_driveSubsystem;

    public CalibrationAutoCommand(){
        m_driveSubsystem = DriveSubsystem.get();
        addRequirements(DriveSubsystem.get());
    }
    @Override
    public void initialize() {

    }   

    @Override
    public void execute() {
        m_driveSubsystem.setSteerMotors(0, 0, 0, 0);
    }
    @Override
    public boolean isFinished() {
        /*double error = m_driveSubsystem.getFrontLeftSteerEncoderPosition() - Math.toRadians(90);
        if (Math.abs(error) < Math.toRadians(1)){
            return true;
        }*/
        return false;
    }
    @Override
    public void cancel() {
        
    }
}