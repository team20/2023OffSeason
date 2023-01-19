// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private PIDController m_frontRightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD, DriveConstants.kSteerPeriod);
  private PIDController m_frontLeftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD, DriveConstants.kSteerPeriod);
  private PIDController m_backRightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD, DriveConstants.kSteerPeriod);
  private PIDController m_backLeftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD, DriveConstants.kSteerPeriod);
  private CANCoder m_frontRightCANCoder = new CANCoder(DriveConstants.kFrontRightCANCoderPort);
  private CANCoder m_frontLeftCANCoder = new CANCoder(DriveConstants.kFrontLeftCANCoderPort);
  private CANCoder m_backRightCANCoder = new CANCoder(DriveConstants.kBackRightCANCoderPort);
  private CANCoder m_backLeftCANCoder = new CANCoder(DriveConstants.kBackLeftCANCoderPort);
  private CANSparkMax m_frontRightDriveMotor = new CANSparkMax(DriveConstants.kFrontRightDrivePort,
      MotorType.kBrushless);
  private CANSparkMax m_frontRightSteerMotor = new CANSparkMax(DriveConstants.kFrontRightSteerPort,
      MotorType.kBrushless);

  private CANSparkMax m_frontLeftDriveMotor = new CANSparkMax(DriveConstants.kFrontLeftDrivePort, MotorType.kBrushless);
  private CANSparkMax m_frontLeftSteerMotor = new CANSparkMax(DriveConstants.kFrontLeftSteerPort, MotorType.kBrushless);

  private CANSparkMax m_backRightDriveMotor = new CANSparkMax(DriveConstants.kBackRightDrivePort, MotorType.kBrushless);
  private CANSparkMax m_backRightSteerMotor = new CANSparkMax(DriveConstants.kBackRightSteerPort, MotorType.kBrushless);

  private CANSparkMax m_backLeftDriveMotor = new CANSparkMax(DriveConstants.kBackLeftDrivePort, MotorType.kBrushless);
  private CANSparkMax m_backLeftSteerMotor = new CANSparkMax(DriveConstants.kBackLeftSteerPort, MotorType.kBrushless);
  private static DriveSubsystem s_subsystem;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Singleton
    if (s_subsystem != null) {
      try {
        throw new Exception("Motor subsystem already initalized!");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    s_subsystem = this;

    // Initialize motors
    {
      m_frontRightDriveMotor.restoreFactoryDefaults();
      m_frontRightDriveMotor.setIdleMode(IdleMode.kBrake);
      m_frontRightDriveMotor.enableVoltageCompensation(12);
      m_frontRightDriveMotor.setSmartCurrentLimit(10);
      m_frontRightDriveMotor.setInverted(DriveConstants.kFrontRightDriveInverted);

      m_frontRightSteerMotor.restoreFactoryDefaults();
      m_frontRightSteerMotor.setIdleMode(IdleMode.kBrake);
      m_frontRightSteerMotor.enableVoltageCompensation(12);
      m_frontRightSteerMotor.setSmartCurrentLimit(10);

      m_frontRightCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

      m_frontRightPIDController.enableContinuousInput(0, 360);

      m_frontLeftDriveMotor.restoreFactoryDefaults();
      m_frontLeftDriveMotor.setIdleMode(IdleMode.kBrake);
      m_frontLeftDriveMotor.enableVoltageCompensation(12);
      m_frontLeftDriveMotor.setSmartCurrentLimit(10);
      m_frontLeftDriveMotor.setInverted(DriveConstants.kFrontLeftDriveInverted);

      m_frontLeftSteerMotor.restoreFactoryDefaults();
      m_frontLeftSteerMotor.setIdleMode(IdleMode.kBrake);
      m_frontLeftSteerMotor.enableVoltageCompensation(12);
      m_frontLeftSteerMotor.setSmartCurrentLimit(10);

      m_frontLeftCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

      m_frontLeftPIDController.enableContinuousInput(0, 360);

      m_backRightDriveMotor.restoreFactoryDefaults();
      m_backRightDriveMotor.setIdleMode(IdleMode.kBrake);
      m_backRightDriveMotor.enableVoltageCompensation(12);
      m_backRightDriveMotor.setSmartCurrentLimit(10);
      m_backRightDriveMotor.setInverted(DriveConstants.kBackRightDriveInverted);

      m_backRightSteerMotor.restoreFactoryDefaults();
      m_backRightSteerMotor.setIdleMode(IdleMode.kBrake);
      m_backRightSteerMotor.enableVoltageCompensation(12);
      m_backRightSteerMotor.setSmartCurrentLimit(10);

      m_backRightCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

      m_backRightPIDController.enableContinuousInput(0, 360);

      m_backLeftDriveMotor.restoreFactoryDefaults();
      m_backLeftDriveMotor.setIdleMode(IdleMode.kBrake);
      m_backLeftDriveMotor.enableVoltageCompensation(12);
      m_backLeftDriveMotor.setSmartCurrentLimit(10);
      m_backLeftDriveMotor.setInverted(DriveConstants.kBackLeftDriveInverted);

      m_backLeftSteerMotor.restoreFactoryDefaults();
      m_backLeftSteerMotor.setIdleMode(IdleMode.kBrake);
      m_backLeftSteerMotor.enableVoltageCompensation(12);
      m_backLeftSteerMotor.setSmartCurrentLimit(10);

      m_backLeftCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

      m_backLeftPIDController.enableContinuousInput(0, 360);
    }
  }

  public static DriveSubsystem get() {
    return s_subsystem;
  }

  public double getHeading() {
    return 0;
  }

  public void setWheelRotationToZeroDegrees() {
    setSteerMotors(0, 0, 0, 0);
  }

  public void setDriveMotors(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed,
      double backRightSpeed) {
    m_frontLeftDriveMotor.set(frontLeftSpeed);
    m_frontRightDriveMotor.set(frontRightSpeed);
    m_backLeftDriveMotor.set(backLeftSpeed);
    m_backRightDriveMotor.set(backRightSpeed);
  }

  public double getFrontLeftSteerEncoderPosition() {
    return m_frontLeftCANCoder.getAbsolutePosition();
  }

  public double getFrontRightSteerEncoderPosition() {
    return m_frontRightCANCoder.getAbsolutePosition();
  }

  public double getBackLeftSteerEncoderPosition() {
    return m_backLeftCANCoder.getAbsolutePosition();

  }

  public double getBackRightSteerEncoderPosition() {
    return m_backRightCANCoder.getAbsolutePosition();
  }

  public void setSteerMotors(double frontLeftAngle, double frontRightAngle, double backLeftAngle,
      double backRightAngle) {
    m_frontLeftPIDController.setSetpoint(frontLeftAngle);
    m_frontRightPIDController.setSetpoint(frontRightAngle);
    m_backLeftPIDController.setSetpoint(backLeftAngle);
    m_backRightPIDController.setSetpoint(backRightAngle);
  }

  @Override
  public void periodic() {
    m_frontLeftSteerMotor.set(m_frontLeftPIDController.calculate(m_frontLeftCANCoder.getAbsolutePosition()));
    m_frontRightSteerMotor.set(m_frontRightPIDController.calculate(m_frontRightCANCoder.getAbsolutePosition()));
    m_backLeftSteerMotor.set(m_backLeftPIDController.calculate(m_backLeftCANCoder.getAbsolutePosition()));
    m_backRightSteerMotor.set(m_backRightPIDController.calculate(m_backRightCANCoder.getAbsolutePosition()));
    SmartDashboard.putNumber("FR CANCoder Rotation", m_frontRightCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("FL CANCoder Rotation", m_frontLeftCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("BR CANCoder Rotation", m_backRightCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("BL CANCoder Rotation", m_backLeftCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("FR PID Setpoint", m_frontRightPIDController.getSetpoint());
    SmartDashboard.putNumber("FL PID Setpoint", m_frontLeftPIDController.getSetpoint());
    SmartDashboard.putNumber("BR PID Setpoint", m_backRightPIDController.getSetpoint());
    SmartDashboard.putNumber("BL PID Setpoint", m_backLeftPIDController.getSetpoint());
  }
}
