// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class CANId {
        public static final int kDriveL1 = 1;
        public static final int kDriveL2 = 2;
        public static final int kDriveR1 = 3;
        public static final int kDriveR2 = 4;
        public static final int kIntake = 5;
        public static final int kArm = 7;

        
    }
    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

            // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final class driveTrain {
        public static final double speedMult = 0.64;
        public static final double rotMult = 0.45;
        //rotation PID gains
        public static final double kRotP = 0.3;
        public static final double kRotI = 0.0;
        public static final double kRotD = 0.0;  

        public static final double maxAutoSpeed = 0.8;
        public static final double minAutoSpeed = -0.8;

        public static final double kS = 0.152;
        public static final double kV = 0.722;
        public static final double kA = 0.0719;
        public static final double rSquared = 0.999;
        public static final double trackWidth = 2.3900506827037105;

        //TODO: set this
        public static final double kPosConversionFactor = 1;
    }

    public static final class intake {
        public static final double kIntakeSpeed = -0.69;
        public static final double kOutakeSpeed = 0.69;
    }

    public static final class arm {
        public static final double kArmUpSpeed = -0.269;
        public static final double kArmDownSpeed = 0.269; 
    }
    

}
