// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class AimToDirection extends Command {
  private Drivetrain m_drivetrain;
  private double m_targetDirectionDegrees;

  private static final double ForwardSpeed = 0.5;
  private static final double TurningSpeed = 0.4;
  private static final double DirectionToleranceDegrees = 20; // plus minus 20 degrees of direction tolerance is ok

  public AimToDirection(Drivetrain drivetrain, double targetDirectionDegrees) {
    m_drivetrain = drivetrain;
    m_targetDirectionDegrees = targetDirectionDegrees;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // are we heading towards target, or we need to turn?
    double currentHeadingDegrees = m_drivetrain.getOdometryAngleDegrees();
    if (currentHeadingDegrees < m_targetDirectionDegrees - DirectionToleranceDegrees) {
      System.out.println("turning right, because heading too far left:" +
        " currentHeading=" + currentHeadingDegrees + ", directionToTarget=" + m_targetDirectionDegrees);
      m_drivetrain.arcadeDrive(0, -TurningSpeed);
    }
    else if (currentHeadingDegrees > m_targetDirectionDegrees + DirectionToleranceDegrees) {
      System.out.println("turning right, because heading too far left:" +
        " currentHeading=" + currentHeadingDegrees + ", directionToTarget=" + m_targetDirectionDegrees);
      m_drivetrain.arcadeDrive(0, TurningSpeed);
    }
    else {
      // we are heading more or less towards the target, let's just go straight
      m_drivetrain.arcadeDrive(ForwardSpeed, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentHeadingDegrees = m_drivetrain.getOdometryAngleDegrees();
    if (currentHeadingDegrees < m_targetDirectionDegrees - DirectionToleranceDegrees)
      return false; // have to turn right, not finished
    else if (currentHeadingDegrees > m_targetDirectionDegrees + DirectionToleranceDegrees)
      return false; // have to turn left, not finished
    else
      return true; // do not need to turn anywhere, finished
  }
}
