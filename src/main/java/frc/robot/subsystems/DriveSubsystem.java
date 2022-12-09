// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private double m_maxSpeed = 0.0;

  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;
  private Joystick m_joystick;
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

    //Inatilize motors
    m_leftMotor = new CANSparkMax(0, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.enableVoltageCompensation(12);
    m_leftMotor.setSmartCurrentLimit(10);

    m_rightMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.enableVoltageCompensation(12);
    m_rightMotor.setSmartCurrentLimit(10);
  }

  public static DriveSubsystem get(){
    return s_subsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double theta = Math.atan2(m_joystick.getY(), m_joystick.getX());
    // Joystick distance from origin
    double r = new Vector2d(m_joystick.getX(), m_joystick.getY()).magnitude(); 
    // Individual motor speeds
    double lSpeed = r * Math.cos(theta) * m_maxSpeed;
    double rSpeed = r * Math.sin(theta) * m_maxSpeed;
    
    m_leftMotor.set(lSpeed);
    m_rightMotor.set(rSpeed);

  }


  public void setMaxSpeed(float maxSpeed){
    m_maxSpeed = maxSpeed;
  }

  public void setJoystick(Joystick joystick) {
    m_joystick = joystick;
  }
}
