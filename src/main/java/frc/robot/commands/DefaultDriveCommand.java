// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Swerve drive joystick command
 * Based on https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
 * and https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf 
 */
public class DefaultDriveCommand extends CommandBase {
  
  private final DriveSubsystem m_driveSubsystem;
  private Supplier<Double> m_yAxisDrive;
  private Supplier<Double> m_xAxisDrive;
  private Supplier<Double> m_rotationAxis;
  private double m_trackWidth;
  private double m_wheelBase;
  private double m_radius;

  public DefaultDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> xAxisDrive, Supplier<Double> yAxisDrive, Supplier<Double> rotationAxis) {
    m_driveSubsystem = driveSubsystem;
    m_yAxisDrive = yAxisDrive;
    m_xAxisDrive = xAxisDrive;
    m_rotationAxis = rotationAxis;
    addRequirements(m_driveSubsystem);
  }

  @Override
  public void initialize() {
    m_trackWidth = DriveConstants.kTrackWidth;
    m_wheelBase = DriveConstants.kWheelBase;
    m_radius = Math.sqrt(m_trackWidth*m_trackWidth + m_wheelBase*m_wheelBase);
  }

  @Override
  public void execute() {

    double fwdSpeed = m_yAxisDrive.get();
    double strSpeed = m_xAxisDrive.get();
    double rotSpeed = m_rotationAxis.get();
    
    double a = strSpeed - rotSpeed * (m_trackWidth/m_radius);
    double b = strSpeed + rotSpeed * (m_trackWidth/m_radius);
    double c = fwdSpeed - rotSpeed * (m_wheelBase/m_radius);
    double d = fwdSpeed + rotSpeed * (m_wheelBase/m_radius);

    double frontRightSpeed = Math.sqrt(b*b + c*c);
    double frontLeftSpeed = Math.sqrt(b*b + d*d);
    double backRightSpeed = Math.sqrt(a*a + d*d);
    double backLeftSpeed = Math.sqrt(a*a + c*c);

    double frontRightAngle = Math.atan2(b, c);
    double frontLeftAngle = Math.atan2(b, d);
    double backRightAngle = Math.atan2(a, d);
    double backLeftAngle = Math.atan2(a, c*c);
    
    m_driveSubsystem.setSteerMotors(frontLeftAngle, frontRightAngle, backLeftAngle, backRightAngle);
    m_driveSubsystem.setDriveMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);

  }
}
