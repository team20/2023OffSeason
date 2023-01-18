// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  }

  @Override 
  public void execute() {
    double fwdSpeed = MathUtil.applyDeadband(m_yAxisDrive.get(), Constants.ControllerConstants.kDeadzone);
    double strSpeed = MathUtil.applyDeadband(m_xAxisDrive.get(), Constants.ControllerConstants.kDeadzone);
    double rotSpeed = MathUtil.applyDeadband(m_rotationAxis.get(), Constants.ControllerConstants.kDeadzone);
    SmartDashboard.putNumber("Foward Speed", fwdSpeed);
    SmartDashboard.putNumber("Strafe Speed", strSpeed);
    SmartDashboard.putNumber("Rotation Speed", rotSpeed);
    double a = strSpeed - rotSpeed * (m_wheelBase/2);
    double b = strSpeed + rotSpeed * (m_wheelBase/2);
    double c = fwdSpeed - rotSpeed * (m_trackWidth/2);
    double d = fwdSpeed + rotSpeed * (m_trackWidth/2);
    SmartDashboard.putNumber("a", a);
    SmartDashboard.putNumber("b", b);
    SmartDashboard.putNumber("c", c);
    SmartDashboard.putNumber("d", d);
    double frontRightSpeed = Math.sqrt(b*b + c*c);
    double frontLeftSpeed = Math.sqrt(b*b + d*d);
    double backRightSpeed = Math.sqrt(a*a + c*c);
    double backLeftSpeed = Math.sqrt(a*a + d*d);
    double maxSpeed = frontRightSpeed;
    if (frontRightSpeed > 1 || frontLeftSpeed > 1 || backRightSpeed > 1 || backLeftSpeed > 1) {
      if (maxSpeed < frontLeftSpeed) {
        maxSpeed = frontLeftSpeed;
      }
      if (maxSpeed < backRightSpeed) {
        maxSpeed = backRightSpeed;
      }
      if (maxSpeed < backLeftSpeed) {
        maxSpeed = backLeftSpeed;
      }
      frontRightSpeed /= maxSpeed;
      frontLeftSpeed /= maxSpeed;
      backRightSpeed /= maxSpeed;
      backLeftSpeed /= maxSpeed;
    }
    SmartDashboard.putNumber("Front Right Wheel Speed", frontRightSpeed);
    SmartDashboard.putNumber("Front Left Wheel Speed", frontLeftSpeed);
    SmartDashboard.putNumber("Back Right Wheel Speed", backRightSpeed);
    SmartDashboard.putNumber("Back Left Wheel Speed", backLeftSpeed);

    // In Radians
    double frontRightAngle = Math.atan2(b, c);
    double frontLeftAngle = Math.atan2(b, d);
    double backRightAngle = Math.atan2(a, c);
    double backLeftAngle = Math.atan2(a, d);
    SmartDashboard.putNumber("Front Right Wheel Angle", frontRightAngle);
    SmartDashboard.putNumber("Front Left Wheel Angle", frontLeftAngle);
    SmartDashboard.putNumber("Back Right Wheel Angle", backRightAngle);
    SmartDashboard.putNumber("Back Left Wheel Angle", backLeftAngle);
    m_driveSubsystem.setSteerMotors(frontLeftAngle, frontRightAngle, backLeftAngle, backRightAngle);
    m_driveSubsystem.setDriveMotors(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);

  }
}
