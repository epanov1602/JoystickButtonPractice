// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.RomiLimelight;
import frc.robot.subsystems.DriveSubsystem;

public class GoToVisualTarget extends Command {
  private final DriveSubsystem m_drivetrain;
  private final RomiLimelight m_camera;
  private final int m_targetPipelineIndex;
  private final double m_desiredTargetSize;
  private final double m_desiredSpeed;
  private final double m_seekingTurnSpeed;
  private int m_previousPipelineIndex = 0;

  public GoToVisualTarget(DriveSubsystem drivetrain, RomiLimelight camera, int targetPipelineIndex, double seekingTurnSpeed, double desiredTargetSize, double desiredSpeed) {
    m_drivetrain = drivetrain;
    m_camera = camera;
    m_targetPipelineIndex = targetPipelineIndex;
    m_seekingTurnSpeed = seekingTurnSpeed;
    m_desiredTargetSize = desiredTargetSize;
    m_desiredSpeed = desiredSpeed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_previousPipelineIndex = m_camera.getPipeline();
    m_camera.setPipeline(m_targetPipelineIndex);
    m_camera.setLEDOn(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     double targetX = m_camera.getX();
    if (targetX == 0) {
      // not seeing the target => keep turning until found it
      m_drivetrain.arcadeDrive(0, m_seekingTurnSpeed);
      return;
    }

    // targetX is pretty much "how many degrees are left to turn"
    double degreesLeftToTurn = -targetX;
    double turningSpeed = Math.abs(degreesLeftToTurn) * Constants.AutoConstants.kRotationStaticGain;
    if (turningSpeed > Constants.AutoConstants.kMaxTurningSpeed)
      turningSpeed = Constants.AutoConstants.kMaxTurningSpeed;
    if (turningSpeed < Constants.AutoConstants.kMinTurningSpeed)
      turningSpeed = Constants.AutoConstants.kMinTurningSpeed;
    if (degreesLeftToTurn > Constants.AutoConstants.kDirectionToleranceDegrees)
      m_drivetrain.arcadeDrive(m_desiredSpeed, turningSpeed);
    else if (degreesLeftToTurn < -Constants.AutoConstants.kDirectionToleranceDegrees)
      m_drivetrain.arcadeDrive(m_desiredSpeed, -turningSpeed);
    else
      m_drivetrain.arcadeDrive(m_desiredSpeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_camera.setLEDOn(false);
    m_camera.setPipeline(m_previousPipelineIndex);
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double targetX = m_camera.getX();
    if (targetX == 0) {
      if (m_seekingTurnSpeed == 0) {
        System.out.println("target lost, but we are not asked to look for it => done going to visual target");
        return true;
      }
      System.out.println("target lost, we keep looking");
      return false;
    }

    double targetSize = m_camera.getA();
    if (targetSize < m_desiredTargetSize) {
      System.out.println("target size " + targetSize + " is smaller than we want (" + m_desiredTargetSize + "), we keep approaching");
      return false;
    }

    // othwerwise, the target is big enough to stop now
    return false;
  }

}
