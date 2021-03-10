// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MyDriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;             //for error reporting
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

import java.nio.file.Path;
import java.io.IOException;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class TrajectoryFollow extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MyDriveTrain locDriveTrain;
  String trajectoryJSON;
  RamseteCommand ramseteCommand;

  /**
   * Creates a new TrajectoryFollow.
   *
   * @param subsystem The subsystem used by this command.
   * @param trajectoryJSONToFollow filePath to pathweaver JSON file in format "dir/path.wpilib.json"
   * |-----| dir - file path to json file from deploy directory (file should be put in deploy directory to be used) 
   * |-----| path - name of pathWeaver path file
   */
  public TrajectoryFollow(MyDriveTrain driveTrain, String trajectoryJSONToFollow) {
    locDriveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);

    //import pathweaver file to trajectory that can be followed
    trajectoryJSON = trajectoryJSONToFollow;
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    
    //initialize ramsete command
    ramseteCommand = new RamseteCommand(
        trajectory,
        locDriveTrain::getPose,
        new RamseteController(Constants.driveTrain.kRamseteB, Constants.driveTrain.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.driveTrain.ksVolts,
        Constants.driveTrain.kvVoltSecondsPerMeter,
        Constants.driveTrain.kaVoltSecondsSquaredPerMeter),
        Constants.driveTrain.kDriveKinematics,
        locDriveTrain::getWheelSpeeds,
        new PIDController(Constants.driveTrain.kPDriveVel, 0, 0),
        new PIDController(Constants.driveTrain.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        locDriveTrain::tankDriveVolts,
        locDriveTrain
    );

    // Reset odometry to the starting pose of the trajectory.
    locDriveTrain.resetOdometry(trajectory.getInitialPose());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ramseteCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ramseteCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ramseteCommand.end(interrupted);
    locDriveTrain.tankDriveVolts(0, 0);     //stop any motion when command finishes
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }
}
