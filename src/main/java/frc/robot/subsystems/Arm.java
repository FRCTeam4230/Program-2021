/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private final TalonSRX armMotor;
  

  public Arm() {

    armMotor = new TalonSRX(Constants.CANId.kArm);

  }

  public void armUp(){
    armMotor.set(ControlMode.PercentOutput, Constants.arm.kArmUpSpeed);
  }

  public void armDown(){
    armMotor.set(ControlMode.PercentOutput, Constants.arm.kArmDownSpeed);
  }

  public void stop() {
    armMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run. why? idk
  }
}
