// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static class IntakeProfile {
        public static int pivotMotorCANID  =   53;
        public static int rollerMotorCANID =   54;

        public static double deployPose     =   0.0;
        public static double retractPose    = -36.0;
        public static double intakePercent  =   0.4;
        public static double outtakePercent =  -0.4;
        public static double poseTolerance  =   2.0;
    }

    public static class LauncherProfile {
        public static int motorACANID       = 55;
        public static int motorBCANID       = 56;
        public static int movingAverageTaps = 25;

        public static double velocityTolerance = 1;
    }

    public static class IndexerProfile {
        public static int    feedMotorCANID  =   52;

        public static double feedPercent     =  1.0;
        public static double backfeedPercent = -1.0;
    }
}
