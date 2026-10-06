// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    // Step 1: setup any motors, sensors, and other variables
    private final TalonFX motorA = new TalonFX(55); // TODO: Change to the actual can ID
    private final TalonFX motorB = new TalonFX(56); // TODO: Change to the actual can ID

    private double launchSpeed = 10.0;

    private LinearFilter averageLauncherSpeed = LinearFilter.movingAverage(25);

    /** Creates a new Launcher. */
    public Launcher() {
        // Step 2: apply motor configs
        configrueMotorA();
        configrueMotorB();
    }

    private void configrueMotorA() {
        // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limts
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Motor direction
        // TODO: Pick one and delete the other
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // PID velocity
        // TODO: This will probably make it move, but tune for your robot
        config.Slot0.kS = 0.1; // Add 0.1 V output to overcome static friction
        config.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        config.Slot0.kP = 0.11; // An error of 1 rps results in 0.11 V output
        config.Slot0.kI = 0; // no output for integrated error
        config.Slot0.kD = 0; // no output for error derivative
        
        // apply the config to the motor
        this.motorA.getConfigurator().apply(config);
    }

    private void configrueMotorB() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // apply the config
        this.motorB.getConfigurator().apply(config);
        
        // Tell this motor to follow whatever the leader does
        this.motorB.setControl(new Follower(this.motorA.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Step 4: smart dashboard and logging
        SmartDashboard.putNumber("LaucherTargetSpeed", this.launchSpeed);
        SmartDashboard.putBoolean("LauncherAtSpeed", this.atTargetSpeed());
        SmartDashboard.putNumber("LauncherSpeed", this.motorA.getVelocity().getValueAsDouble());
    }

    // Step 3: interal functions, logic, etc.

    /**
    * Sets motorA's velocity to the value
    * @param vel, Rotations / second
    */
    public void setMotorAVel(double velocity) {
        // Basic TalonFX Velocity Control
        this.motorA.setControl(new VelocityVoltage(velocity));
    }

    /**
     * Tells us if the subsystem is at its target speed
     * @return true if the launcher is at its target speed, otherwise false
     */
    public boolean atTargetSpeed() {
        return this.averageLauncherSpeed.calculate(this.motorA.getClosedLoopError().getValueAsDouble()) < 1;
    }

    /**
     * A command to enable the launcher at the subsytem's speed, disables when done
     * @return a command to enable the subsysem.
     */
    public Command enableLauncher() {
        return new StartEndCommand(
            () -> this.setMotorAVel(this.launchSpeed), 
            () -> this.motorA.stopMotor(), 
            this
        );
    }

    /**
     * A command to change the subsystem speed
     * @param change the change to the speed in rotations / second
     * @return a command to change the speed
     */
    public Command bumpSpeed(double change) {
        return Commands.runOnce(() -> this.launchSpeed += change);
    }
}
