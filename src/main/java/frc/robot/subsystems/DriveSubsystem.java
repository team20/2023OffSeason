// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private PIDController m_frontLeftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD, DriveConstants.kSteerPeriod);
  private PIDController m_frontRightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD, DriveConstants.kSteerPeriod);
  private PIDController m_backLeftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD, DriveConstants.kSteerPeriod);
  private PIDController m_backRightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD, DriveConstants.kSteerPeriod);
  private CANCoder m_frontLeftCANCoder = new CANCoder(DriveConstants.kFrontLeftCANCoderPort);
  private CANCoder m_frontRightCANCoder = new CANCoder(DriveConstants.kFrontRightCANCoderPort);
  private CANCoder m_backLeftCANCoder = new CANCoder(DriveConstants.kBackLeftCANCoderPort);
  private CANCoder m_backRightCANCoder = new CANCoder(DriveConstants.kBackRightCANCoderPort);

  private CANSparkMax m_frontLeftDriveMotor = new CANSparkMax(DriveConstants.kFrontLeftDrivePort, MotorType.kBrushless);
  public RelativeEncoder m_frontLeftDriveEncoder = m_frontLeftDriveMotor.getEncoder();
  private CANSparkMax m_frontLeftSteerMotor = new CANSparkMax(DriveConstants.kFrontLeftSteerPort, MotorType.kBrushless);
  private CANSparkMax m_frontRightDriveMotor = new CANSparkMax(DriveConstants.kFrontRightDrivePort,
      MotorType.kBrushless);
  public RelativeEncoder m_frontRightDriveEncoder = m_frontRightDriveMotor.getEncoder();
  private CANSparkMax m_frontRightSteerMotor = new CANSparkMax(DriveConstants.kFrontRightSteerPort,
      MotorType.kBrushless);
  private CANSparkMax m_backLeftDriveMotor = new CANSparkMax(DriveConstants.kBackLeftDrivePort, MotorType.kBrushless);
  public RelativeEncoder m_backLeftDriveEncoder = m_backLeftDriveMotor.getEncoder();
  private CANSparkMax m_backLeftSteerMotor = new CANSparkMax(DriveConstants.kBackLeftSteerPort, MotorType.kBrushless);
  private CANSparkMax m_backRightDriveMotor = new CANSparkMax(DriveConstants.kBackRightDrivePort, MotorType.kBrushless);
  public RelativeEncoder m_backRightDriveEncoder = m_backRightDriveMotor.getEncoder();
  private CANSparkMax m_backRightSteerMotor = new CANSparkMax(DriveConstants.kBackRightSteerPort, MotorType.kBrushless);
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

    // Initialize motors, PID controllers
    {
      configMotorController(m_frontLeftDriveMotor);
      m_frontLeftDriveMotor.setInverted(DriveConstants.kFrontLeftDriveInverted);
      configMotorController(m_frontLeftSteerMotor);

      m_frontLeftPIDController.enableContinuousInput(0, 360);

      configMotorController(m_frontRightDriveMotor);
      m_frontRightDriveMotor.setInverted(DriveConstants.kFrontRightDriveInverted);
      configMotorController(m_frontRightSteerMotor);

      m_frontRightPIDController.enableContinuousInput(0, 360);

      configMotorController(m_backLeftDriveMotor);
      m_backLeftDriveMotor.setInverted(DriveConstants.kBackLeftDriveInverted);
      configMotorController(m_backLeftSteerMotor);

      m_backLeftPIDController.enableContinuousInput(0, 360);

      configMotorController(m_backRightDriveMotor);
      m_backRightDriveMotor.setInverted(DriveConstants.kBackRightDriveInverted);
      configMotorController(m_backRightSteerMotor);

      m_backRightPIDController.enableContinuousInput(0, 360);
    }
  }

  public static DriveSubsystem get() {
    return s_subsystem;
  }

  /***
   * Configures our motors with the exact same settings
   * 
   * @param motorController
   *                        The CANSparkMax to configure
   */
  public static void configMotorController(CANSparkMax motorController) {
    motorController.restoreFactoryDefaults();
    motorController.setIdleMode(IdleMode.kBrake);
    motorController.enableVoltageCompensation(12);
    motorController.setSmartCurrentLimit(10);
  }

  public double getHeading() {
    return 0;
  }

  public void setWheelRotationToZeroDegrees() {
    setSteerMotors(0, 0, 0, 0);
  }

  /**
   * Makes our drive motors spin at the specified speeds
   * 
   * @param frontLeftSpeed
   *                        Speed of the front left wheel in duty cycles [-1, 1]
   * @param frontRightSpeed
   *                        Speed of the front right wheel in duty cycles [-1, 1]
   * @param backLeftSpeed
   *                        Speed of the back left wheel in duty cycles [-1, 1]
   * @param backRightSpeed
   *                        Speed of the back right wheel in duty cycles [-1, 1]
   */
  public void setDriveMotors(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed,
      double backRightSpeed) {
    m_frontLeftDriveMotor.set(frontLeftSpeed);
    m_frontRightDriveMotor.set(frontRightSpeed);
    m_backLeftDriveMotor.set(backLeftSpeed);
    m_backRightDriveMotor.set(backRightSpeed);
  }

  /***
   * Sets the target angles in degrees for each wheel on the robot
   * 
   * @param frontLeftAngle
   *                        The target angle of the front left wheel in degrees
   * @param frontRightAngle
   *                        The target angle of the front right wheel in degrees
   * @param backLeftAngle
   *                        The target angle of the back left wheel in degrees
   * @param backRightAngle
   *                        The target angle of the back right wheel in degrees
   */
  public void setSteerMotors(double frontLeftAngle, double frontRightAngle, double backLeftAngle,
      double backRightAngle) {
    m_frontLeftPIDController.setSetpoint(frontLeftAngle);
    m_frontRightPIDController.setSetpoint(frontRightAngle);
    m_backLeftPIDController.setSetpoint(backLeftAngle);
    m_backRightPIDController.setSetpoint(backRightAngle);
  }

  /***
   * Recalculates the PID output, and uses it to drive our steer motors. Also logs
   * wheel rotations, and PID setpoints
   */
  @Override
  public void periodic() {
    // For each of our steer motors, feed the current angle of the wheel into its
    // PID controller, and use it to calculate the duty cycle for its motor, and
    // spin the motor
    m_frontLeftSteerMotor.set(m_frontLeftPIDController.calculate(m_frontLeftCANCoder.getAbsolutePosition()));
    m_frontRightSteerMotor.set(m_frontRightPIDController.calculate(m_frontRightCANCoder.getAbsolutePosition()));
    m_backLeftSteerMotor.set(m_backLeftPIDController.calculate(m_backLeftCANCoder.getAbsolutePosition()));
    m_backRightSteerMotor.set(m_backRightPIDController.calculate(m_backRightCANCoder.getAbsolutePosition()));
    // Logging
    SmartDashboard.putNumber("FL CANCoder Rotation", m_frontLeftCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("FR CANCoder Rotation", m_frontRightCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("BL CANCoder Rotation", m_backLeftCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("BR CANCoder Rotation", m_backRightCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("FL PID Setpoint", m_frontLeftPIDController.getSetpoint());
    SmartDashboard.putNumber("FR PID Setpoint", m_frontRightPIDController.getSetpoint());
    SmartDashboard.putNumber("BL PID Setpoint", m_backLeftPIDController.getSetpoint());
    SmartDashboard.putNumber("BR PID Setpoint", m_backRightPIDController.getSetpoint());
  }
}
