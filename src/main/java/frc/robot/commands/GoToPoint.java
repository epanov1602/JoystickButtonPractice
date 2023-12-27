// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class GoToPoint extends CommandBase {
  private Drivetrain m_drivetrain;
  private double m_targetX, m_targetY, m_distanceTolerance;

  private static final double ForwardSpeed = 0.35;
  private static final double TurningSpeed = 0.2;
  private static final double DirectionToleranceDegrees = 20; // plus minus 20 degrees of direction tolerance is ok

  /** Creates a new GoToTarget. */
  public GoToPoint(Drivetrain drivetrain, double targetX, double targetY, double distanceTolerance) {
    m_drivetrain = drivetrain;
    m_targetX = targetX;
    m_targetY = targetY;
    m_distanceTolerance = distanceTolerance;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentXY = m_drivetrain.getOdometryPosition();
  
    // is our target north of us, east of us, or what?
    double xToTarget = m_targetX - currentXY.getX(); // positive X-to-target means we need to be moving further North, negative means we need to move a bit South
    double yToTarget = m_targetY - currentXY.getY(); // positive Y-to-target means we need to be moving further East, negative means further West

    // so what should be our heading direction, based on x-to-target and y-to-target?
    Rotation2d directionToTarget = new Rotation2d(xToTarget, yToTarget);
    double directionToTargetDegrees = directionToTarget.getDegrees(); /* for example, -88 degrees would mean that the target is almost directly West of us */

    // do we need to know distance to target to slow down before we overshoot it or something?
    double distanceToTarget = getDistanceToTarget();

    // are we heading towards target, or we need to turn?
    double currentHeadingDegrees = m_drivetrain.getOdometryAngleDegrees();
    if (currentHeadingDegrees < directionToTargetDegrees - DirectionToleranceDegrees) {
      System.out.println("turning right, because heading too far left:" +
        " currentHeading=" + currentHeadingDegrees + ", directionToTarget=" + directionToTarget.getDegrees() + ", distanceToTarget=" + distanceToTarget);
      m_drivetrain.arcadeDrive(0, -TurningSpeed);
    }
    else if (currentHeadingDegrees > directionToTargetDegrees + DirectionToleranceDegrees) {
      System.out.println("turning right, because heading too far left:" +
        " currentHeading=" + currentHeadingDegrees + ", directionToTarget=" + directionToTarget.getDegrees() + ", distanceToTarget=" + distanceToTarget);
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
    double distance = getDistanceToTarget();
    if (distance < m_distanceTolerance)
      return true; // if we are close enough to the target, assume we reached it 
    else
      return false;
  }

  // how far are we from the target
  private double getDistanceToTarget() {
    Pose2d currentXY = m_drivetrain.getOdometryPosition();
    double xDistance = currentXY.getX() - m_targetX;
    double yDistance = currentXY.getY() - m_targetY;
    double totalDistance = Math.sqrt(xDistance * xDistance + yDistance * yDistance);
    return totalDistance;
  }
}
