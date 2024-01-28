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
import frc.robot.commands.TurnDegrees;
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

    Command rotateRightTwoTimes = new TurnDegrees(0.8, 720, m_drivetrain);
    JoystickButton button4 = new JoystickButton(m_joystick, 4);
    button4.onTrue(rotateRightTwoTimes);

    m_autonomousCommand = rotateRightTwoTimes;

    Command goNorthAndTurnSouth = new SequentialCommandGroup(
      new GoToPoint(m_drivetrain, 40, 10, 2),
      new AimToDirection(m_drivetrain, 90)
    );
    JoystickButton button2 = new JoystickButton(m_joystick, 2);
    button2.onTrue(goNorthAndTurnSouth);

    Command triangularRace = new SequentialCommandGroup(
      new GoToPoint(m_drivetrain, 40, 0, 2),
      new GoToPoint(m_drivetrain, 40, 40, 2),
      new GoToPoint(m_drivetrain, 0, 0, 2),
      new GoToPoint(m_drivetrain, 40, 0, 2),
      new GoToPoint(m_drivetrain, 40, 40, 2),
      new GoToPoint(m_drivetrain, 0, 0, 2),
      new AimToDirection(m_drivetrain, 0)
    );

    m_autonomousCommand = triangularRace;

    JoystickButton button3 = new JoystickButton(m_joystick, 3);
    button3.onTrue(triangularRace);

  }

  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  public RobotContainer() {
    initialize();
    m_drivetrain.setDefaultCommand(m_teleopCommand);
  }
}
