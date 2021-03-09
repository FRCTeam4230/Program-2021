// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import com.kauailabs.navx.frc.AHRS;

public class MyDriveTrain extends SubsystemBase {
  /** Creates a new MyDriveTrain. */
  
  private final CANSparkMax l1Motor = new CANSparkMax(Constants.CANId.kDriveL1, MotorType.kBrushless);
  private final CANSparkMax m_left2 = new CANSparkMax(Constants.CANId.kDriveL2, MotorType.kBrushless);
  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(l1Motor, m_left2);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(new CANSparkMax(Constants.CANId.kDriveR1, MotorType.kBrushless), new CANSparkMax(Constants.CANId.kDriveR2, MotorType.kBrushless));
  private final DifferentialDrive driveSys = new DifferentialDrive(leftGroup, rightGroup);

  private CANEncoder l1Encoder;
  // NavX class thing
  private AHRS ahrs;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public MyDriveTrain() {
    //try to set up connection to NavX, otherwise throw an error
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    ahrs = new AHRS(SPI.Port.kMXP); 
    } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
    ahrs.reset();
    l1Encoder = l1Motor.getEncoder(EncoderType.kQuadrature, 4096);
    m_odometry = new DifferentialDriveOdometry(ahrs);
  }

  /**drive, but speed is limited to Constants.drivetrain.max/minAutoSpeed
   * squaredInputs is set to false
   */
  public void drive(double speed, double rotation){
    this.drive(speed, rotation, true, false);
  }

  /** drive without max/minAutoSpeed limits
   * @param speed double from -1 to 1
   * @param rotation double from -1 to 1
   * @param limits false if speed and rotation shouldn't be limited to Constants.drivetrain.min/maxAutospeed.
   * --Defaults to true
   * @param squaredInputs whether or not arcadeDrive() squares it's inputs
  */
  public void drive(double speed, double rotation, Boolean limits, Boolean squaredInputs){
    if (limits) {
      driveSys.arcadeDrive(MathUtil.clamp(speed, Constants.driveTrain.minAutoSpeed, Constants.driveTrain.maxAutoSpeed),
        MathUtil.clamp(rotation, Constants.driveTrain.minAutoSpeed, Constants.driveTrain.maxAutoSpeed), squaredInputs);
    } else {
      driveSys.arcadeDrive(speed, rotation, squaredInputs);
    }
  }

  public double getEncoderPosition()
  {
    return m_frontLeft.getEncoder().getPosition();
  }

  public double getHeading(){
    return ahrs.getAngle();
  }

  public void resetHeading(){
    ahrs.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
