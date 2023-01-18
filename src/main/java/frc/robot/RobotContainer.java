// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ResetToZeroDegreesCommand;
import frc.robot.subsystems.DriveSubsystem; 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick m_joystick = new Joystick(Constants.ControllerConstants.kDriverControllerPort);
  private final GenericHID m_controller = new GenericHID(Constants.ControllerConstants.kDriverControllerPort);
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driveSubsystem.setDefaultCommand(
      new DefaultDriveCommand(
        m_driveSubsystem, () -> m_joystick.getRawAxis(Axis.kLeftX),
        ()->m_joystick.getRawAxis(Axis.kLeftY),
        ()->m_joystick.getRawAxis(Axis.kRightX)));
    new Trigger(() ->m_controller.getRawButton(Constants.ControllerConstants.Button.kTriangle)).onTrue(new ResetToZeroDegreesCommand());
    
  }
}
