// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.RomiGyro;

public class DriveSubsystem extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeter = .35;// 70 mm is the actual diameter, scale down by 5

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);
  
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(0), new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
  
  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final RomiGyro m_romiGyro = new RomiGyro();

  public static DriveSubsystem s_subsystem;
  /** Creates a new RomiDrivetrain. */
  public DriveSubsystem() {
    s_subsystem = this;
    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public static DriveSubsystem get(){
    return s_subsystem;
  }
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }


  public double getHeading(){
    return (m_romiGyro.getAngle())*Math.PI/180;
  }

  public Pose2d getPose(){
    //System.out.println(getHeading());

    SmartDashboard.putString("pose", m_odometry.getPoseMeters().toString());
    return m_odometry.getPoseMeters();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(getHeading());
    //System.out.println(getLeftDistance());
    //System.out.println(getRightDistance());
    m_odometry.update(new Rotation2d(getHeading()), getLeftDistance(), getRightDistance());
    //System.out.println(m_odometry.getPoseMeters().toString());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
