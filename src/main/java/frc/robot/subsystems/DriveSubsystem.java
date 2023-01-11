// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax m_frontRightDriveMotor = new CANSparkMax(DriveConstants.kFrontRightDrivePort, MotorType.kBrushless);
  private CANSparkMax m_frontRightSteerMotor = new CANSparkMax(DriveConstants.kFrontRightSteerPort, MotorType.kBrushless);  

  private CANSparkMax m_frontLeftDriveMotor = new CANSparkMax(DriveConstants.kFrontLeftDrivePort, MotorType.kBrushless);
  private CANSparkMax m_frontLeftSteerMotor = new CANSparkMax(DriveConstants.kFrontLeftSteerPort, MotorType.kBrushless);  
  
  private CANSparkMax m_backRightDriveMotor = new CANSparkMax(DriveConstants.kBackRightDrivePort, MotorType.kBrushless);
  private CANSparkMax m_backRightSteerMotor = new CANSparkMax(DriveConstants.kBackRightSteerPort, MotorType.kBrushless);

  private CANSparkMax m_backLeftDriveMotor = new CANSparkMax(DriveConstants.kBackLeftDrivePort, MotorType.kBrushless);
  private CANSparkMax m_backLeftSteerMotor = new CANSparkMax(DriveConstants.kBackLeftSteerPort, MotorType.kBrushless);

  private SparkMaxPIDController m_frontRightSteerPIDController = m_frontRightSteerMotor.getPIDController();
  private SparkMaxPIDController m_frontLeftSteerPIDController = m_frontLeftSteerMotor.getPIDController();
  private SparkMaxPIDController m_backRightSteerPIDController = m_backRightSteerMotor.getPIDController();
  private SparkMaxPIDController m_backLeftSteerPIDController = m_backLeftSteerMotor.getPIDController();

  private RelativeEncoder m_frontRightSteerEncoder = m_frontRightSteerMotor.getEncoder();
  private RelativeEncoder m_frontLeftSteerEncoder = m_frontLeftSteerMotor.getEncoder();
  private RelativeEncoder m_backRightSteerEncoder = m_backRightSteerMotor.getEncoder();
  private RelativeEncoder m_backLeftSteerEncoder = m_backLeftSteerMotor.getEncoder();

  
  private static DriveSubsystem s_subsystem;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    //Singleton
    if(s_subsystem != null){
      try {
        throw new Exception("Motor subsystem already initalized!");
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
    s_subsystem = this;

    //Initialize motors
    m_frontRightDriveMotor.restoreFactoryDefaults();
    m_frontRightDriveMotor.setIdleMode(IdleMode.kBrake);
    m_frontRightDriveMotor.enableVoltageCompensation(12);
    m_frontRightDriveMotor.setSmartCurrentLimit(10);

    m_frontRightSteerMotor.restoreFactoryDefaults();
    m_frontRightSteerMotor.setIdleMode(IdleMode.kBrake);
    m_frontRightSteerMotor.enableVoltageCompensation(12);
    m_frontRightSteerMotor.setSmartCurrentLimit(10);
    
    m_frontRightSteerPIDController.setP(DriveConstants.kP);
    m_frontRightSteerPIDController.setI(DriveConstants.kI);
    m_frontRightSteerPIDController.setIZone(DriveConstants.kIz);
    m_frontRightSteerPIDController.setD(DriveConstants.kD);
    m_frontRightSteerPIDController.setFF(DriveConstants.kFF);
    m_frontRightSteerPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);

    m_frontRightSteerPIDController.setSmartMotionAccelStrategy(DriveConstants.kTrapezoidal, DriveConstants.kSlotID);
    m_frontRightSteerPIDController.setSmartMotionMaxAccel(DriveConstants.kMaxAcel, DriveConstants.kSlotID);
    m_frontRightSteerPIDController.setSmartMotionMaxVelocity(DriveConstants.kMaxVelocity, DriveConstants.kSlotID);
    m_frontRightSteerPIDController.setSmartMotionAllowedClosedLoopError(DriveConstants.kAllowedError, DriveConstants.kSlotID);
    m_frontRightSteerPIDController.setSmartMotionMinOutputVelocity(DriveConstants.kMinVelocity, DriveConstants.kSlotID);

    m_frontLeftDriveMotor.restoreFactoryDefaults();
    m_frontLeftDriveMotor.setIdleMode(IdleMode.kBrake);
    m_frontLeftDriveMotor.enableVoltageCompensation(12);
    m_frontLeftDriveMotor.setSmartCurrentLimit(10);

    m_frontLeftSteerMotor.restoreFactoryDefaults();
    m_frontLeftSteerMotor.setIdleMode(IdleMode.kBrake);
    m_frontLeftSteerMotor.enableVoltageCompensation(12);
    m_frontLeftSteerMotor.setSmartCurrentLimit(10);
    
    m_frontLeftSteerPIDController.setP(DriveConstants.kP);
    m_frontLeftSteerPIDController.setI(DriveConstants.kI);
    m_frontLeftSteerPIDController.setIZone(DriveConstants.kIz);
    m_frontLeftSteerPIDController.setD(DriveConstants.kD);
    m_frontLeftSteerPIDController.setFF(DriveConstants.kFF);
    m_frontLeftSteerPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);

    m_frontLeftSteerPIDController.setSmartMotionAccelStrategy(DriveConstants.kTrapezoidal, DriveConstants.kSlotID);
    m_frontLeftSteerPIDController.setSmartMotionMaxAccel(DriveConstants.kMaxAcel, DriveConstants.kSlotID);
    m_frontLeftSteerPIDController.setSmartMotionMaxVelocity(DriveConstants.kMaxVelocity, DriveConstants.kSlotID);
    m_frontLeftSteerPIDController.setSmartMotionAllowedClosedLoopError(DriveConstants.kAllowedError, DriveConstants.kSlotID);
    m_frontLeftSteerPIDController.setSmartMotionMinOutputVelocity(DriveConstants.kMinVelocity, DriveConstants.kSlotID);

    m_backRightDriveMotor.restoreFactoryDefaults();
    m_backRightDriveMotor.setIdleMode(IdleMode.kBrake);
    m_backRightDriveMotor.enableVoltageCompensation(12);
    m_backRightDriveMotor.setSmartCurrentLimit(10);

    m_backRightSteerMotor.restoreFactoryDefaults();
    m_backRightSteerMotor.setIdleMode(IdleMode.kBrake);
    m_backRightSteerMotor.enableVoltageCompensation(12);
    m_backRightSteerMotor.setSmartCurrentLimit(10);
    
    m_backRightSteerPIDController.setP(DriveConstants.kP);
    m_backRightSteerPIDController.setI(DriveConstants.kI);
    m_backRightSteerPIDController.setIZone(DriveConstants.kIz);
    m_backRightSteerPIDController.setD(DriveConstants.kD);
    m_backRightSteerPIDController.setFF(DriveConstants.kFF);
    m_backRightSteerPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);

    m_backRightSteerPIDController.setSmartMotionAccelStrategy(DriveConstants.kTrapezoidal, DriveConstants.kSlotID);
    m_backRightSteerPIDController.setSmartMotionMaxAccel(DriveConstants.kMaxAcel, DriveConstants.kSlotID);
    m_backRightSteerPIDController.setSmartMotionMaxVelocity(DriveConstants.kMaxVelocity, DriveConstants.kSlotID);
    m_backRightSteerPIDController.setSmartMotionAllowedClosedLoopError(DriveConstants.kAllowedError, DriveConstants.kSlotID);
    m_backRightSteerPIDController.setSmartMotionMinOutputVelocity(DriveConstants.kMinVelocity, DriveConstants.kSlotID);

    m_backLeftDriveMotor.restoreFactoryDefaults();
    m_backLeftDriveMotor.setIdleMode(IdleMode.kBrake);
    m_backLeftDriveMotor.enableVoltageCompensation(12);
    m_backLeftDriveMotor.setSmartCurrentLimit(10);

    m_backLeftSteerMotor.restoreFactoryDefaults();
    m_backLeftSteerMotor.setIdleMode(IdleMode.kBrake);
    m_backLeftSteerMotor.enableVoltageCompensation(12);
    m_backLeftSteerMotor.setSmartCurrentLimit(10);
    
    m_backLeftSteerPIDController.setP(DriveConstants.kP);
    m_backLeftSteerPIDController.setI(DriveConstants.kI);
    m_backLeftSteerPIDController.setIZone(DriveConstants.kIz);
    m_backLeftSteerPIDController.setD(DriveConstants.kD);
    m_backLeftSteerPIDController.setFF(DriveConstants.kFF);
    m_backLeftSteerPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);

    m_backLeftSteerPIDController.setSmartMotionAccelStrategy(DriveConstants.kTrapezoidal, DriveConstants.kSlotID);
    m_backLeftSteerPIDController.setSmartMotionMaxAccel(DriveConstants.kMaxAcel, DriveConstants.kSlotID);
    m_backLeftSteerPIDController.setSmartMotionMaxVelocity(DriveConstants.kMaxVelocity, DriveConstants.kSlotID);
    m_backLeftSteerPIDController.setSmartMotionAllowedClosedLoopError(DriveConstants.kAllowedError, DriveConstants.kSlotID);
    m_backLeftSteerPIDController.setSmartMotionMinOutputVelocity(DriveConstants.kMinVelocity, DriveConstants.kSlotID);
  }

  public static DriveSubsystem get(){
    return s_subsystem;
  }

  public double getHeading(){
    return 0;
  }

  public void setDriveMotors(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed){
    m_frontLeftDriveMotor.set(frontLeftSpeed);
    m_frontRightDriveMotor.set(frontRightSpeed);
    m_backLeftDriveMotor.set(backLeftSpeed);
    m_backRightDriveMotor.set(backRightSpeed);
  }
  public double getFrontLeftSteerEncoderPosition(){
    return m_frontLeftSteerEncoder.getPosition();
  }
  public double getFrontRightSteerEncoderPosition(){
    return m_frontRightSteerEncoder.getPosition();
  }
  public double getBackLeftSteerEncoderPosition(){
    return m_backLeftSteerEncoder.getPosition();
  }
  public double getBackRightSteerEncoderPosition(){
    return m_backRightSteerEncoder.getPosition();
  }
  private double getTargetEncoderPosition(double currentEncoderPosition, double targetAngle){
    return currentEncoderPosition - (42/(2*Math.PI))*((currentEncoderPosition%42)*((2*Math.PI)/42) - targetAngle);
  }
  public void setSteerMotors(double frontLeftAngle, double frontRightAngle, double backLeftAngle, double backRightAngle){
    m_frontLeftSteerPIDController.setReference(getTargetEncoderPosition(getFrontLeftSteerEncoderPosition(), frontLeftAngle), ControlType.kPosition);
    m_frontRightSteerPIDController.setReference(getTargetEncoderPosition(getFrontRightSteerEncoderPosition(), frontRightAngle), ControlType.kPosition);
    m_backLeftSteerPIDController.setReference(getTargetEncoderPosition(getBackLeftSteerEncoderPosition(), backLeftAngle), ControlType.kPosition);
    m_backRightSteerPIDController.setReference(getTargetEncoderPosition(getBackRightSteerEncoderPosition(), backRightAngle), ControlType.kPosition);
  }
  @Override
  public void periodic() {
    

  }
}
