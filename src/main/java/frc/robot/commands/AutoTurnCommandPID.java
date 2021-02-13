// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// example code from https://pdocs.kauailabs.com/navx-mxp/examples/rotate-to-angle-2/ is used here

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.driveTrain;
import frc.robot.subsystems.MyDriveTrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;

public class AutoTurnCommandPID extends CommandBase {
  /** Creates a new AutoTurnCommandPID. */

  MyDriveTrain locDriveTrain;
  Double locRSpeed = 0.0;
  Double initialAngle = 0.0;
  Double locAngle = 0.0;        //angle to rotate to in degrees
  
  // NavX class thing
  AHRS ahrs;

  // tolerance for error in PID -- the closer this is to 0 the longer turning will take
  static final double kToleranceDegrees = 2.0f;
  // Creates a PIDController with gains kP, kI, and kD
  // more info on PID control https://frc-pdr.readthedocs.io/en/latest/control/pid_control.html
  PIDController pid = new PIDController(driveTrain.kRotP, driveTrain.kRotI, driveTrain.kRotD);

  //Rotate to angle "angle"
  //speed controled by "Constants.driveTrain.kRotP" and distance from target
  //uses a PID loop to hopefully make it more accurate
  public AutoTurnCommandPID(MyDriveTrain driveTrain, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    locDriveTrain = driveTrain;
    locAngle = angle;


    //try to set up connection to NavX, otherwise throw an error
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //initialize NavX
    ahrs.reset();

    //initialize PID
    pid.setSetpoint(locAngle);
    pid.setTolerance(kToleranceDegrees);
    pid.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //rotate at speed given by the PID loop
    locDriveTrain.drive(0.0, pid.calculate(ahrs.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop any motion
    locDriveTrain.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
