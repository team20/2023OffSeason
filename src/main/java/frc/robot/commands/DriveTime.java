// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTime extends CommandBase {
  private Instant m_startTime;
  /** Creates a new DriveTime. */
  public DriveTime() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveSubsystem.get());
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.get().arcadeDrive(0.5, 0);
    m_startTime = Instant.now();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.get().getPose();
    DriveSubsystem.get().arcadeDrive(0.5, -0.1);

    //System.out.println(DriveSubsystem.get().getRightDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.get().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Duration.between(m_startTime, Instant.now()).toMillis() > 5000;
  }
}
