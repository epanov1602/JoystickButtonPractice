// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.AimToDirection;
import frc.robot.commands.AimToVisualTarget;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.GoToPoint;
import frc.robot.commands.GoToVisualTarget;
import frc.robot.commands.ResetOdometry;
import frc.robot.sensors.RomiLimelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {

  /* what will robot do in teleop and auto mode? */
  private Command m_autonomousCommand = null;
  private Command m_teleopCommand = null;

  /* what this robot consists of? */
  private final Drivetrain m_drivetrain = new Drivetrain();
  //private final RomiLimelight m_camera = new RomiLimelight();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  private final Joystick m_joystick = new Joystick(0);


  /* in this "initialize" function we setup the commands for this robot */
  private void initialize() {
    // the default TELEOP command is "arcade drive" controlled by joystick axis 5 (drive speed) and axis 0 (turn speed)
    Command followJoystickSticks = new ArcadeDrive(m_drivetrain, () -> -m_joystick.getRawAxis(5), () -> -m_joystick.getRawAxis(0));
    m_teleopCommand = followJoystickSticks;

    // the default AUTONOMOUS command is to go forward by 30 inches
    Command goForward30Inches = new DriveDistance(0.5, 30, m_drivetrain);
    m_autonomousCommand = goForward30Inches;

    // a command for joystick button 1 is for robot to go to the scoring location at X=30, Y=-10
    Command goToScoringPoint = new GoToPoint(m_drivetrain, 30, -10, 3); 
    JoystickButton button1 = new JoystickButton(m_joystick, 1);
    button1.onTrue(goToScoringPoint);
  }

  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  public RobotContainer() {
    initialize();
    m_drivetrain.setDefaultCommand(m_teleopCommand);
  }
}
