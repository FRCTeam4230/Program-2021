// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MyDriveTrain;

public class AutoDrive extends CommandBase {

  MyDriveTrain locDriveTrain;
  Double locSpeed = 0.0;
  Double initialPosition = 0.0;
  Double locDistance = 0.0;

  /** Creates a new AutoDrive. */
  public AutoDrive(MyDriveTrain driveTrain, double speed, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    locDriveTrain = driveTrain;
    locSpeed = speed;
    locDistance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = Math.abs(locDriveTrain.getEncoderPosition());
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    locDriveTrain.drive(locSpeed, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    locSpeed = 0.0;
    locDriveTrain.drive(locSpeed, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (locDistance < Math.abs(locDriveTrain.getEncoderPosition()) - initialPosition) {
      return true;
    }
    return false;
  }
}
