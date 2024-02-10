// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.AimToDirection;
import frc.robot.commands.ArcadeDrive;

import frc.robot.commands.GoToPoint;
import frc.robot.commands.MockPickupCommand;
import frc.robot.commands.MockShootCommand;
import frc.robot.commands.ResetOdometry;
//import frc.robot.sensors.RomiLimelight;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.OnBoardIO;
//import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;


public class RobotContainer {

  /* what will robot do in teleop and auto mode? */
  private Command m_autonomousCommand = null;
  private Command m_teleopCommand = null;

  /* what this robot consists of? */
  private final DriveSubsystem m_drivetrain = new DriveSubsystem();
  //private final RomiLimelight m_camera = new RomiLimelight();
  //private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  private final Joystick m_joystick0 = new Joystick(0);


  /* in this "initialize" function we setup the commands for this robot */
  private void initialize() {
    // the default TELEOP command is "arcade drive" controlled by joystick axis 5 (drive speed) and axis 0 (turn speed)
    Command followJoystickSticks = new ArcadeDrive(m_drivetrain, () -> -m_joystick0.getRawAxis(5), () -> -m_joystick0.getRawAxis(0));
    m_teleopCommand = followJoystickSticks;

    // the default AUTONOMOUS command is to go forward by 40 inches
    Command goToPointNorth = new GoToPoint(m_drivetrain, 40, 0, false);
    m_autonomousCommand = goToPointNorth;
  }

  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  public RobotContainer() {
    initialize();
    m_drivetrain.setDefaultCommand(m_teleopCommand);
  }
}
