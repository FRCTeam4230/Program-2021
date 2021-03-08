// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MainAutoCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MyDriveTrain;
import frc.robot.commands.MyTeleOpDriveCommand;
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

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MyDriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;             //for error reporting
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

import java.nio.file.Path;
import java.io.IOException;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final MyDriveTrain myDriveTrain = new MyDriveTrain();
  private final XboxController driver1 = new XboxController(0);

 
  private final MyTeleOpDriveCommand m_driveCommand = new MyTeleOpDriveCommand(myDriveTrain, driver1);

  private final MainAutoCommand autoCommand = new MainAutoCommand(myDriveTrain);

 // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    myDriveTrain.setDefaultCommand(m_driveCommand);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_robotDrive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter,
        Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
