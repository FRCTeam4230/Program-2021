// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANId {
        public static final int kDriveL1 = 1;
        public static final int kDriveL2 = 2;
        public static final int kDriveR1 = 3;
        public static final int kDriveR2 = 4;
        public static final int kIntake = 5;
        public static final int kArm = 7;
    }

    public static final class driveTrain {
        public static final double speedMult = 0.64;
        public static final double rotMult = 0.45;
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
