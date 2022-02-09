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
        /** 
         * DriveUtil Constants
         */
        public static final int LEFT_PRIMARY = 1;
        public static final int LEFT_SECONDARY = 2;
        public static final int RIGHT_PRIMARY = 3;
        public static final int RIGHT_SECONDARY = 4;

        public static final double AUTO_TURN_SPEED = 0.25;

        public static final double TICKS_PER_INCH = 2267.4512;
        public static final double AUTO_DRIVE_SPEED = 0.3;

        public static final double DRIVER_P = 0.00075;
        public static final double DRIVER_I = 0.0;
        public static final double DRIVER_D = 0.0;
        public static final double DRIVER_F = 0.0;
        public static final double DRIVER_DEADBAND = 500;

        /**
         * CargoUtil Constants
         */
        public static final int BALL_MAGNET = 6;
        public static final int LOW_INDEXER = 7;
        public static final int HIGH_INDEXER = 8;
        public static final int SHOOTER = 9;
        public static final int LIMIT_SWTICH = 0;

        public static final double BALL_MAGNET_OUTPUT = 0.4;
        public static final double INDEXER_OUTPUT = 0.4;
        public static final double SHOOTER_RPM = 3000.0;

        public static final double SHOOTER_P = 0.0;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.0;
        public static final double SHOOTER_F = 0.0;

        public static final int ARCADE_LEFT_DAMPENING = 60;
        public static final int ARCADE_RIGHT_DAMPENING = 60;

        /**
         * Controller Input Device Mapping
         */
        //public static final int LEFT_STICK = 2;
        //public static final int RIGHT_STICK = 1;
        public static final int XBOX_DRIVER = 0;
        public static final int XBOX_OPERATOR = 1;

        /**
         * Controller Button Mapping
         */
        public static final int kLeftXAxisNum = 0;
        public static final int kLeftYAxisNum = 1;
        public static final int kRightXAxisNum = 2;
        public static final int kRightYAxisNum = 3;

        public static final int kXButtonNum = 1;
        public static final int kAButtonNum = 2;
        public static final int kBButtonNum = 3;
        public static final int kYButtonNum = 4;
        public static final int kLeftBumperNum = 5;
        public static final int kRightBumperNum = 6;
        public static final int kLeftTriggerNum = 7;
        public static final int kRightTriggerNum = 8;
        public static final int kBackButtonNum = 9;
        public static final int kStartButtonNum = 10;
        public static final int kLeftStickButtonNum = 11;
        public static final int kRightStickButtonNum = 12;

}
