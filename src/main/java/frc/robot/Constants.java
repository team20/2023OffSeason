// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.SparkMaxPIDController.AccelStrategy;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.1;
		public static final double kTriggerDeadzone = .05;

		public static final class Axis {
			public static final int kLeftX = 0;
			public static final int kLeftY = 1;
			public static final int kRightX = 2;
			public static final int kLeftTrigger = 3;
			public static final int kRightTrigger = 4;
			public static final int kRightY = 5;
		}

		public static final class Button {
			public static final int kSquare = 1;
			public static final int kX = 2;
			public static final int kCircle = 3;
			public static final int kTriangle = 4;
			public static final int kLeftBumper = 5;
			public static final int kRightBumper = 6;
			public static final int kShare = 9;
			public static final int kOptions = 10;
			public static final int kLeftStick = 11;
			public static final int kRightStick = 12;
			public static final int kPS = 13;
			public static final int kTrackpad = 14;
		}

		public static final class DPad {
			public static final int kUp = 0;
			public static final int kRight = 90;
			public static final int kDown = 180;
			public static final int kLeft = 270;
		}
	}

    public static final class DriveConstants {
        public static final int kFrontLeftPort = 0;
        public static final int kFrontRightPort = 0;
        public static final int kBackLeftPort = 0;
        public static final int kBackRightPort = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kIz = 0;
        public static final double kFF = 0;
        public static final double kMinOutput = 0;
        public static final double kMaxOutput = 0;
        public static final AccelStrategy kTrapezoidal = null;
        public static final int kSlotID = 0;
        public static final double kMaxAcel = 0;
        public static final double kMaxVelocity = 0;
        public static final double kAllowedError = 0;
        public static final double kMinVelocity = 0;
        public static final double kWheelBase = 0;
        public static final double kTrackWidth = 0;
    }
}
