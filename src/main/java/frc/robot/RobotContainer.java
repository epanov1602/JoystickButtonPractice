// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AimToDirection;
import frc.robot.commands.AimToVisualTarget;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // commands
  private Command m_autonomousCommand = null;

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final RomiLimelight m_camera = new RomiLimelight();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channel 0
  private final Joystick m_controller = new Joystick(0);

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

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
    // default command is arcade drive.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // setup a command to run when button 1 is pressed
    Command goNorthWest42inches = new GoToPoint(m_drivetrain, 30, -30, 3);
    JoystickButton button1 = new JoystickButton(m_controller, 1);
    button1.onTrue(goNorthWest42inches);

    // setup a command to reset odometry, attached to joystick button 2
    JoystickButton button2 = new JoystickButton(m_controller, 2);
    button2.onTrue(new ResetOdometry(m_drivetrain));

    // setup a command to aim to target setup in Limelight pipeline 2 (toy car), and attach that command attached to joystick button 3
    Command aimToToyCar = new AimToVisualTarget(m_drivetrain, m_camera, 2);
    JoystickButton button3 = new JoystickButton(m_controller, 3);
    button3.onTrue(aimToToyCar);
    
    // setup a command to race a triangular race course when button 3 is pressed
    JoystickButton button4 = new JoystickButton(m_controller, 4);
    Command driveBlindAlongTriangleAndComeBackHome = new SequentialCommandGroup(
      new ResetOdometry(m_drivetrain),
      new GoToPoint(m_drivetrain, 30, -30, 1),
      new GoToPoint(m_drivetrain, 30, 30, 1),
      new GoToPoint(m_drivetrain, 0, 0, 1)
    );
    button4.onTrue(driveBlindAlongTriangleAndComeBackHome);

    // setup a command to go to where the toy car approximately is, find it there, and then approach it
    Command longRoutine = new SequentialCommandGroup(
      new GoToPoint(m_drivetrain, 40, 40, 2),
      new PrintCommand("went to point, looking for toy car"),
      new AimToVisualTarget(m_drivetrain, m_camera, 2),
      new PrintCommand("found a toy car"),
      new GoToVisualTarget(m_drivetrain, m_camera, 2, 1.5),
      new PrintCommand("approached the toy car"),
      new PrintCommand("grabbed toy car"),
      new GoToPoint(m_drivetrain, 40, 40, 2),
      new PrintCommand("went to point, looking for apriltag"),
      new AimToVisualTarget(m_drivetrain, m_camera, 3),
      new PrintCommand("located apriltag"),
      new GoToVisualTarget(m_drivetrain, m_camera, 3, 6),
      new PrintCommand("arrived at apriltag")
    );

    m_autonomousCommand = longRoutine;
  }

  private String message(String string) {
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomousCommand;
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(5), () -> -m_controller.getRawAxis(0));
  }
}
