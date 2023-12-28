// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiLimelight;
import frc.robot.subsystems.Drivetrain;

public class GoToVisualTarget extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final RomiLimelight m_camera;
  private final int m_targetPipelineIndex;
  private final double m_desiredTargetSize;

  private static final double TargetXTolerance = 5; // plus minus five pixels is fine
  private static final double SeekingTurnSpeed = 0.3;
  private static final double MaxTurnSpeed = 0.5;
  private static final double MaxForwardSpeed = 0.5;

  public GoToVisualTarget(Drivetrain drivetrain, RomiLimelight camera, int targetPipelineIndex, double desiredTargetSize) {
    m_drivetrain = drivetrain;
    m_camera = camera;
    m_targetPipelineIndex = targetPipelineIndex;
    m_desiredTargetSize = desiredTargetSize;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_camera.setPipeline(m_targetPipelineIndex);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetX = m_camera.getX();
    if (targetX == 0) {
      m_drivetrain.arcadeDrive(0, SeekingTurnSpeed);
      // we don't see the target, let's do nothing; isFinished() will return true and this command will finish
    }
    else {
      // we see the target, we can approach it
      double targetSize = m_camera.getA();

      // turning: if X is negative, we want to turn right; if X is positive, we want to turn left
      double turnSpeed = -targetX * SeekingTurnSpeed / 30;
      if (turnSpeed > MaxTurnSpeed)
        turnSpeed = MaxTurnSpeed;
      else if (turnSpeed < -MaxTurnSpeed)
        turnSpeed = -MaxTurnSpeed;

      // approaching: if target looks small, we can come closer (otherwise better not)
      double forwardSpeed = 0;
      if (targetSize < m_desiredTargetSize) {
        forwardSpeed = MaxForwardSpeed;
        if (targetSize > 0.5 * targetSize)
          forwardSpeed = MaxForwardSpeed / 3; // if target is pretty close already, go slower
      }

      m_drivetrain.arcadeDrive(forwardSpeed, turnSpeed);
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
    double targetX = m_camera.getX();
    if (targetX == 0) {
      System.out.println("target lost");
      return false; // if target is lost, we lost it => finished
    }

    if (-TargetXTolerance < targetX && targetX < TargetXTolerance) {
      // if target is pretty close to center of the screen, just check that we are at good distance to it
      double targetSize = m_camera.getA();
      if (targetSize != 0 && targetSize >= m_desiredTargetSize) {
      System.out.println("reached destination because correct target size"+targetSize);

        return true; // good distance too => we are finished
      }
    }
    // othwerwise, not there yet
    return false;
  }

}
