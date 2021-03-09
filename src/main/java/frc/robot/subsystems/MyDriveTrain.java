// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import com.revrobotics.EncoderType;

import com.kauailabs.navx.frc.AHRS;

public class MyDriveTrain extends SubsystemBase {
  /** Creates a new MyDriveTrain. */
  
  private CANSparkMax l1Motor = new CANSparkMax(Constants.CANId.kDriveL1, MotorType.kBrushless);
  private CANSparkMax l2Motor = new CANSparkMax(Constants.CANId.kDriveL2, MotorType.kBrushless);
  private CANSparkMax r1Motor = new CANSparkMax(Constants.CANId.kDriveR1, MotorType.kBrushless);
  private CANSparkMax r2Motor = new CANSparkMax(Constants.CANId.kDriveR2, MotorType.kBrushless);
  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(l1Motor, l2Motor);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(r1Motor, r2Motor);
  private final DifferentialDrive driveSys = new DifferentialDrive(leftGroup, rightGroup);

  private CANEncoder l1Encoder;
  private CANEncoder l2Encoder;
  private CANEncoder r1Encoder;
  private CANEncoder r2Encoder;
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
    l1Encoder.setPositionConversionFactor(Constants.driveTrain.kPosConversionFactor);
    l2Encoder = l2Motor.getEncoder(EncoderType.kQuadrature, 4096);
    l2Encoder.setPositionConversionFactor(Constants.driveTrain.kPosConversionFactor);
    r1Encoder = r1Motor.getEncoder(EncoderType.kQuadrature, 4096);
    r1Encoder.setPositionConversionFactor(Constants.driveTrain.kPosConversionFactor);
    r2Encoder = r2Motor.getEncoder(EncoderType.kQuadrature, 4096);
    r2Encoder.setPositionConversionFactor(Constants.driveTrain.kPosConversionFactor);
    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(ahrs.getRotation2d());
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(l1Encoder.getVelocity(), r1Encoder.getVelocity());
  }

  /**drive, but speed is limited to Constants.drivetrain.max/minAutoSpeed
   * squaredInputs is set to false
   */
  public void drive(double speed, double rotation){
    this.drive(speed, rotation, true, false);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, ahrs.getRotation2d());
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

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts);
    rightGroup.setVoltage(-rightVolts);
    driveSys.feed();
  }

  private void resetEncoders(){
    l1Encoder.setPosition(0);
    l2Encoder.setPosition(0);
    r1Encoder.setPosition(0);
    r2Encoder.setPosition(0);
  }

  public double getEncoderPosition()
  {
    return 0.0;
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (l1Encoder.getPosition() + r1Encoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return l1Encoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return r1Encoder;
  }

  public double getHeading(){
    return ahrs.getAngle();
  }

  public void resetHeading(){
    ahrs.reset();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -ahrs.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(ahrs.getRotation2d(), l1Encoder.getPosition(),
    r1Encoder.getPosition());
  }
}
