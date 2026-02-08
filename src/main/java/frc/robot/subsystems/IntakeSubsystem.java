// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.Preset;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem INSTANCE;
  private TalonFX intakeMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID, "rio");
  private TalonFX pivotMotor = new TalonFX(Constants.IntakeConstants.pivotMotorID, "rio");

  @SuppressWarnings("WeakerAccess")
  public static IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  private SmartMotorControllerConfig intakeSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new ArmFeedforward(0, 0, 0))
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25)); // PID Controller, Max Velocity, Max Acceleration;

  private TalonFXWrapper intakeSmartMotorController = new TalonFXWrapper(intakeMotor, DCMotor.getKrakenX60(1),
      intakeSmcConfig);

  private PivotConfig pivotConfig = new PivotConfig(intakeSmartMotorController)
      .withSoftLimits(Preset.Intake.position, Preset.Stowed.position)
      .withHardLimit(Preset.Intake.position, Preset.Stowed.position)
      .withStartingPosition(Preset.Stowed.position)
      .withTelemetry("Arm", TelemetryVerbosity.HIGH);

  private Pivot intakePivot = new Pivot(pivotConfig);

  // Set angle of Intake, but command and Intake does not stop
  public Command setAngle(Angle angle) {
    return intakePivot.run(angle);
  }

  // public Command setAngleAndStop(Angle angle){
  // return intakePivot.runTo(angle);
  // }

  // Closed Loop controller for Intake
  public void setIntakeSetpoint(Angle angle) {
    intakePivot.setMechanismPositionSetpoint(angle);
  }

  // dutycycle
  public Command setIntakeSpeed(double dutycycle) {
    return intakePivot.set(dutycycle);
  }

  public Command sysId() {
    return intakePivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakePivot.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intakePivot.simIterate();
  }
}
