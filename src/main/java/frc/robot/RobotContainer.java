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

    // create a command to reset odometry, and bind it to joystign button Y 
    Command resetOdometry = new ResetOdometry(m_drivetrain);
    JoystickButton buttonY = new JoystickButton(m_joystick0, Button.kY.value);
    buttonY.onTrue(resetOdometry);

    // create a command for bagel pickup and assign it to button B
    Command pickUpBagel = new MockPickupCommand(m_drivetrain);
    JoystickButton buttonB = new JoystickButton(m_joystick0, Button.kB.value);
    buttonB.onTrue(pickUpBagel);

    // create a more useful autonomous command
    Command turnAroundToAim = new AimToDirection(m_drivetrain, -135);
    Command shoot = new MockShootCommand(m_drivetrain);
    Command goToBagel2 = new GoToPoint(m_drivetrain, 30, -10, false);
    Command pickUpBagel2 = new MockPickupCommand(m_drivetrain);
    Command goBackToSpeaker = new GoToPoint(m_drivetrain, 10, -20, false);
    Command aimAgain = new AimToDirection(m_drivetrain, -180);
    Command shootAgain = new MockShootCommand(m_drivetrain);
    Command bigAutonomousRoutine = new SequentialCommandGroup(turnAroundToAim, shoot, goToBagel2, pickUpBagel2, goBackToSpeaker, aimAgain, shootAgain);
    m_autonomousCommand = bigAutonomousRoutine;

    // create a command for following the left side of the field and turning to the speaker to shoot, and assign it to the left bumper
    Command goToLeftSide = new GoToPoint(m_drivetrain, 60, -10, true);
    Command turnSouth1 = new AimToDirection(m_drivetrain, 180);
    Command goToLeftSideCorner = new GoToPoint(m_drivetrain, 10, -10, true);
    Command turnEast = new AimToDirection(m_drivetrain, -90);
    Command goToMiddleLeft = new GoToPoint(m_drivetrain, 10, -40, true);
    Command aimToSpeaker1 = new AimToDirection(m_drivetrain, 180);
    Command shoot1 = new MockShootCommand(m_drivetrain);
    Command leftSideFetchRoutine = new SequentialCommandGroup(goToLeftSide, turnSouth1, goToLeftSideCorner, turnEast, goToMiddleLeft, aimToSpeaker1, shoot1);
    leftSideFetchRoutine = leftSideFetchRoutine.until(() -> m_joystick0.getRawButtonPressed(Button.kX.value) == true);

    JoystickButton leftBumper = new JoystickButton(m_joystick0, Button.kLeftBumper.value);
    leftBumper.onTrue(leftSideFetchRoutine);
  }

  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  public RobotContainer() {
    initialize();
    m_drivetrain.setDefaultCommand(m_teleopCommand);
  }
}
