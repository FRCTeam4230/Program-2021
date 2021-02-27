// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.MyDriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;             //for error reporting
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

import java.nio.file.Path;
import java.io.IOException;

/** An example command that uses an example subsystem. */
public class TrajectoryFollow extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MyDriveTrain m_subsystem;
  String trajectoryJSON;

  /**
   * Creates a new TrajectoryFollow.
   *
   * @param subsystem The subsystem used by this command.
   * @param trajectoryJSONToFollow filePath to pathweaver JSON file in format "dir/path.wpilib.json"
   * |-----| dir - file path to json file from deploy directory (file should be put in deploy directory to be used) 
   * |-----| path - name of pathWeaver path file
   */
  public TrajectoryFollow(MyDriveTrain subsystem, String trajectoryJSONToFollow) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    //import pathweaver file to trajectory that can be followed
    trajectoryJSON = trajectoryJSONToFollow;
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
