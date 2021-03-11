// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MyDriveTrain;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.subsystems.Intake;

public class MyTeleOpDriveCommand extends CommandBase {
  /** Creates a new MyTeleOpDriveCommand. */
  MyDriveTrain locDriveTrain;
  XboxController locDriverJoyStick;
  Intake m_intake;


  public MyTeleOpDriveCommand(MyDriveTrain driveTrain, XboxController driverJoystick, Intake intake) {
    locDriveTrain = driveTrain;
    locDriverJoyStick = driverJoystick;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    locDriveTrain.drive(locDriverJoyStick.getY(GenericHID.Hand.kLeft)*Constants.driveTrain.speedMult,
                        locDriverJoyStick.getX(GenericHID.Hand.kRight)*Constants.driveTrain.rotMult, false, true);
                        
    if (locDriverJoyStick.getRawButtonPressed(5)) {
      m_intake.outtakeRoller();
    } else if (locDriverJoyStick.getRawButtonPressed(7)) {
      m_intake.intakeRoller();
    } else {
      m_intake.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      locDriveTrain.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
