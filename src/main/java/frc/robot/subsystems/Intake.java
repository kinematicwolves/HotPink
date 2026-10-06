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
    private final TalonFX pivotMotor  = new TalonFX(53); // TODO: Change to the actual can ID
    private final TalonFX rollerMotor = new TalonFX(54); // TODO: Change to the actual can ID

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
        
        // Current Limits
        // TODO: Adjust as needed for your robot
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Neutral mode
        // TODO: Pick one and delete the other
        // config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Motor direction
        // TODO: Test and select one, delete the other.
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // PID
        // TODO: Tune for your robot
        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        
        // Soft limits
        // TODO: Determine positional limits, set, and enable
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        
        // apply the config
        this.pivotMotor.getConfigurator().apply(config);
    }

    private void configureRoller() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current Limits
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 20;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // Neutral mode
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Motor direction
        // TODO: Test and select one, delete the other
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // apply the config
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
    * Sets rollerMotor's speed to the value
    * @param speed, -1 to 1
    */
    public void setRollerMotorPercent(double speed) {
        // Basic TalonFX make it spin
        this.rollerMotor.set(speed);
    }

    /**
    * Sets pivotMotor's position to the value
    * @param position, Rotations / second
    */
    public void setPivotMotorPose(double position) {
        // Basic TalonFX Position Control
        this.pivotMotor.setControl(new PositionVoltage(position));
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
        return Commands.run(() -> this.setPivotMotorPose(this.retractPose), this).until(() -> this.pivotAtPose());
    }

    /**
     * Depolys the intake 
     * @return a command to deploy the intake
     */
    public Command Deploy() {
        return Commands.run(() -> this.setPivotMotorPose(this.deployPose), this).until(() -> this.pivotAtPose());
    }

    /**
     * Runs the intake forwad to intake game pieces
     * @return a command to feed game pieces
     */
    public Command Feed() {
        return new StartEndCommand(
            () -> setRollerMotorPercent(1),
            () -> setRollerMotorPercent(0),
            this
        );
    }

    /**
     * Runs the intake in reverse to dump game pieces
     * @return a command to un-feed game pieces
     */
    public Command UnFeed() {
        return new StartEndCommand(
            () -> setRollerMotorPercent(-1),
            () -> setRollerMotorPercent(0),
            this
        );

    }
}
