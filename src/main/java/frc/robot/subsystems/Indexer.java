// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerProfile;

public class Indexer extends SubsystemBase {
    // Step 1: create motors, sensors, etc for the subsysem
    // setting up motors
    private SparkMax feedMotor = new SparkMax(IndexerProfile.feedMotorCANID, MotorType.kBrushless);

    // internal variables
    private double feedPercent = 0.0;

    /** Creates a new Indexer. */
    public Indexer() {
        // Step 2: configure motors
        configureFeedMotor();
    }

    private void configureFeedMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        // current limits
        config.smartCurrentLimit(30);

        // Neutral mode
        config.idleMode(IdleMode.kCoast);

        // Motor direction
        config.inverted(true);

        // apply the config
        this.feedMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Step 4: display on smartdashboard and log values
        SmartDashboard.putNumber("FeedPercet", this.feedPercent);
    }

    // Step 3: cerating interal functions, commands, etc.
    private void setPercents(double feedPercent) {
        // saving my settings so i can see them later
        this.feedPercent = feedPercent;

        // applying the speeds to the motors
        feedMotor.set(feedPercent);
    }

    /**
     * Feeds game pices by spining the motors forward
     * @return A command to feed game pices to the launcher
     */
    public Command Feed() {
        return new StartEndCommand(
            () -> this.setPercents(IndexerProfile.feedPercent),
            () -> this.setPercents(0),
            this
        );
    }

    /**
     * Und game pices by spining the motors backwards
     * @return A command to unfeed game pices from the launcher
     */
    public Command BackFeed() {
        return new StartEndCommand(
            () -> this.setPercents(IndexerProfile.backfeedPercent),
            () -> this.setPercents(0),
            this
        );
    }
}
