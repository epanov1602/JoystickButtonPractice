// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.RomiLimelight;
import frc.robot.subsystems.DriveSubsystem;

public class AimToVisualTarget extends Command {

  private final DriveSubsystem m_drivetrain;
  private final RomiLimelight m_camera;
  private final int m_targetPipelineIndex;
  private final double m_seekingTurnSpeed;

  private int m_previousPipelineIndex = 0;

  public AimToVisualTarget(DriveSubsystem drivetrain, RomiLimelight camera, int targetPipelineIndex, double seekingTurnSpeed) {
    m_drivetrain = drivetrain;
    m_camera = camera;
    m_targetPipelineIndex = targetPipelineIndex;
    m_seekingTurnSpeed = seekingTurnSpeed;
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
      m_drivetrain.arcadeDrive(0, turningSpeed);
    else if (degreesLeftToTurn < -Constants.AutoConstants.kDirectionToleranceDegrees)
      m_drivetrain.arcadeDrive(0, -turningSpeed);
    else
      m_drivetrain.arcadeDrive(0, 0);
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
      System.out.println("target is not acquired, we keep looking");
      return false; // if target is not acquired, we are not finished
    }
    double degreesLeftToTurn = -targetX;

    // are we facing good angle?
    if (Math.abs(degreesLeftToTurn) > Constants.AutoConstants.kDirectionToleranceDegrees) {
      System.out.println("still busy with " + degreesLeftToTurn + " degrees left to turn to aim into target");
      return false;
    }

    // we are at good angle, but is the chassis stopped? (i.e. won't overshoot the turn)
    ChassisSpeeds chassisSpeeds = m_drivetrain.getChassisSpeeds();
    double turningSpeed = chassisSpeeds.omegaRadiansPerSecond * (180.0 / Math.PI);
    if (Math.abs(turningSpeed) > Constants.AutoConstants.kTurningSpeedToleranceDegreesPerSecond) {
      System.out.println("turning speed " + turningSpeed + " degrees per second when aiming into target");
      return false;
    }

    // if we are here, we are aimed and the chassis is almost not moving 
    return true;
  }
}
