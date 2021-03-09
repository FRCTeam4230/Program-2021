// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// example code from https://pdocs.kauailabs.com/navx-mxp/examples/rotate-to-angle-2/ is used here

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MyDriveTrain;

public class AutoTurnCommand extends CommandBase {
  /** Creates a new AutoTurnCommand. */

  MyDriveTrain locDriveTrain;
  Double locRSpeed = 0.0;
  Double initialAngle = 0.0;
  Double locAngle = 0.0;        //angle to rotate to in degrees

  public AutoTurnCommand(MyDriveTrain DriveTrain, double rSpeed, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveTrain);
    locDriveTrain = DriveTrain;
    locRSpeed = rSpeed;
    locAngle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = locDriveTrain.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    locDriveTrain.drive(0.0, locRSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    locDriveTrain.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((locDriveTrain.getHeading() - initialAngle) >= locAngle);
  }
}
