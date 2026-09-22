// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // Step 1: Create motros, sensors, etc.
    private final TalonFX pivotMotor  = new TalonFX(53);
    private final TalonFX rollerMotor = new TalonFX(54);

    // internal variables
    // TODO: Test intake motion and set accordingly
    private final double deployPose  = 0.0;
    private final double retractPose = 0.0;

    /** Creates a new Intake. */
    public Intake() {
        // Step 2: do configuraitons
        configurePivot();
        configureRoller();
    }

    private void configurePivot() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // for a pivot, i configure the following
        // current limits
        config.CurrentLimits.StatorCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // direction
        // TODO: Test and pick the correct one 
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // idle mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; 
        
        // position limits / soft limits
        // TODO: Test and enter good values
        // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        
        // pid
        // TODO: test and set kp
        config.Slot0.kP = 0; // set to be 1 / totoal sensor range 
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        // apply the config to my motor
        this.pivotMotor.getConfigurator().apply(config);

        this.pivotMotor.setPosition(0.0);
    }

    private void configureRoller() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // curent limits
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // direction
        // TODO: Test and pick the correct one
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // idle mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // apply my config to my motor
        this.rollerMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Step 4: display on smartdashboard and log stuff
        SmartDashboard.putNumber("PivotPose", this.pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("PivotAtPose", this.pivotAtPose());
    }

    // Step 3: cerate interal functions, commands, etc
    /**
     * Sets the roller percetn
     * @param percet a value between -1 and 1
     */
    private void setRollerPercent(double percet) {
        this.rollerMotor.set(percet);
    }

    /**
     * Sets the pivot position in encoder counts
     * @param pose the encoder count to move to
     */
    private void setPivotPose(double pose) {
        this.pivotMotor.setControl(new PositionVoltage(pose).withSlot(0));
    }

    /**
     * Tells us if the pivot is at its position or not
     * @return True if pivot is at its position, othewise false
     */
    private boolean pivotAtPose() {
        return this.pivotMotor.getClosedLoopError().getValue() < 0.1;
    }

    /**
     * Retracts the intake
     * @return a command to rectract the intake
     */
    public Command Retract() {
        return Commands.run(() -> this.setPivotPose(this.retractPose), this).until(() -> this.pivotAtPose());
    }

    /**
     * Depolys the intake 
     * @return a command to deploy the intake
     */
    public Command Deploy() {
        return Commands.run(() -> this.setPivotPose(this.deployPose), this).until(() -> this.pivotAtPose());
    }

    /**
     * Runs the intake forwad to intake game pieces
     * @return a command to feed game pieces
     */
    public Command Feed() {
        return new StartEndCommand(
            () -> setRollerPercent(1),
            () -> setRollerPercent(0),
            this
        );
    }

    /**
     * Runs the intake in reverse to dump game pieces
     * @return a command to un-feed game pieces
     */
    public Command UnFeed() {
        return new StartEndCommand(
            () -> setRollerPercent(-1),
            () -> setRollerPercent(0),
            this
        );

    }
}
