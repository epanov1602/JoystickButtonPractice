// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.RomiLimelight;
import frc.robot.subsystems.Drivetrain;

public class AimToVisualTarget extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final RomiLimelight m_camera;
  private final int m_targetPipelineIndex;

  private static final double TargetXTolerance = 5; // plus minus five pixels is fine
  private static final double SeekingTurnSpeed = 0.15;
  private static final double MaxTurnSpeed = 0.4;

  public AimToVisualTarget(Drivetrain drivetrain, RomiLimelight camera, int targetPipelineIndex) {
    m_drivetrain = drivetrain;
    m_camera = camera;
    m_targetPipelineIndex = targetPipelineIndex;
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
      // not seeing the target => keep turning until found it
      m_drivetrain.arcadeDrive(0, SeekingTurnSpeed);
    } else {
      System.out.println("target at x=" + targetX);
      // if X is negative, we want to turn right; if X is positive, we want to turn left
      double turnSpeed = -targetX * SeekingTurnSpeed / 15;
      // 30 is the maximum value of X in Limelight: so if X is already closer than 30, we want to be turning slower than SeekingTurnSpeed (to avoid overshooting)
      if (turnSpeed > MaxTurnSpeed)
        turnSpeed = MaxTurnSpeed;
      else if (turnSpeed < -MaxTurnSpeed)
        turnSpeed = -MaxTurnSpeed;
      m_drivetrain.arcadeDrive(0, turnSpeed);
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
    if (targetX == 0)
      return false; // if target is not acquired, we are not finished
    else if (-TargetXTolerance < targetX && targetX < TargetXTolerance) {
      System.out.print("target acquired at x=" + targetX);
      return true; // if target is pretty close to center of the screen, we are finished
    } else
      return false; // otherwise, we are not finished yet
  }
}
