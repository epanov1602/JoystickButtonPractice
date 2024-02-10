// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class DriveConstants {
        // initial position of robot on the field (does not have to be {x=0, y=0})
        public static final double kInitialX = 10;
        public static final double kInitialY = 10;

        // how to convert wheel speed into turning speeds (depends how far apart the wheels are)
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(6); // 6 inches between wheels
    }

    // constants to tune for this specific chassis
    public final class AutoConstants {
        // turning
        public static final double kRotationStaticGain = 0.006;
        public static final double kMaxTurningSpeed = 0.99;
        public static final double kMinTurningSpeed = 0.07; // any value lower than this causes motors to not spin at all
        public static final double kDirectionToleranceDegrees = 3; // plus minus 5 degrees of direction tolerance is ok
        public static final double kTurningSpeedToleranceDegreesPerSecond = 3; // if the chassis is moving slower than this and facing the right way, we can stop aiming
      
        // driving
        public static final double kForwardStaticGain = 0.08;
        public static final double kMaxForwardSpeed = 0.99;
        public static final double kMinForwardSpeed = 0.07; // any value lower than this causes motors to not spin at all
        public static final double kDistanceTolerance = 2.0; // inches
        public static final double kForwardSpeedTolerance = 0.9; // inches per second
    }
}
