// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.SmartMotorController;
import com.ctre.phoenix6.hardware.CANcoder;

public class ShooterSubsystem extends SubsystemBase {
  // Motors
  private TalonFX shooterFlywheelMotor = new TalonFX(Constants.ShooterConstants.shooterFlywheelMotorID,
      CANBus.roboRIO());
  private TalonFX shooterPitchMotor = new TalonFX(Constants.ShooterConstants.shooterPitchMotorID, CANBus.roboRIO());
  private CANcoder pitchEncoder = new CANcoder(Constants.ShooterConstants.shooterThroughboreEncoderID);

  // SMC Configs
  private SmartMotorControllerConfig shooterFlywheelSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.002772, 0, 0.001, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(1, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new SimpleMotorFeedforward(0, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withTelemetry("Shooter Flywheel Motor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(Constants.ShooterConstants.shooterFlywheelGearRatio)) // 1:1 Gear Ratio
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorControllerConfig shooterPitchSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45)) // PID Controller,
                                                                                                    // Max Velocity, Max
                                                                                                    // Acceleration;
      .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new ArmFeedforward(0, 0.05, 0))
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      .withTelemetry("Shooter Pitch Motor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(Constants.ShooterConstants.shooterPitchGearRatio)) // 12:1 Gear Ratio
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withExternalEncoder(pitchEncoder)
      .withExternalEncoderInverted(false)
      .withExternalEncoderGearing(Constants.ShooterConstants.shooterPitchGearRatio) // Gear Ratio for Encoder
      .withExternalEncoderZeroOffset(Constants.ShooterConstants.shooterThroughboreEncoderOffset)
      .withUseExternalFeedbackEncoder(true);

  // Smart Motor Controllers
  private SmartMotorController shooterFlywheelMotorController = new TalonFXWrapper(shooterFlywheelMotor,
      DCMotor.getKrakenX60(1), shooterFlywheelSmcConfig);
  private SmartMotorController shooterPitchMotorController = new TalonFXWrapper(shooterPitchMotor,
      DCMotor.getKrakenX60(1), shooterPitchSmcConfig);

  // FlyWheel Config
  private final FlyWheelConfig shooterFlyWheelConfig = new FlyWheelConfig(shooterFlywheelMotorController)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.3))
      .withUpperSoftLimit(RPM.of(1000))
      .withTelemetry("Shooter Flywheel Mechanism", TelemetryVerbosity.HIGH);

  private PivotConfig shooterPitchConfig = new PivotConfig(shooterPitchMotorController)
      .withSoftLimits(Degrees.of(-20), Degrees.of(10))
      .withHardLimit(Degrees.of(-20), Degrees.of(10)) // Only for sim
      .withStartingPosition(Degrees.of(-5))
      .withMOI(Constants.IntakeConstants.intakeCenterOfMassFromPivot, Constants.IntakeConstants.intakeMass) // Arbitrary
                                                                                                            // Values
                                                                                                            // for Sim
      .withTelemetry("Shooter Pitch", TelemetryVerbosity.HIGH);

  // Mechanisms
  private FlyWheel shooterFlywheel = new FlyWheel(shooterFlyWheelConfig);
  private Pivot shooterPitch = new Pivot(shooterPitchConfig);

  // Commands
  public AngularVelocity getShooterFlywheelVelocity() {
    return shooterFlywheel.getSpeed();
  }

  public Command setShooterFlywheelVelocity(AngularVelocity speed) {
    return shooterFlywheel.run(speed);
  }

  public void setShooterFlywheelVelocitySetpoint(AngularVelocity speed) {
    shooterFlywheel.setMechanismVelocitySetpoint(speed);
  }

  public Command setShooterFlywheelDutyCycle(double dutyCycle) {
    return shooterFlywheel.set(dutyCycle);
  }

  // DO NOT USE UNLESS YOU KNOW WHAT THIS DOES!
  public Command setPitchAngle(Angle angle) {
    return shooterPitch.run(angle);
  }

  // USE THIS FOR CLOSED LOOP CONTROL AS MAIN METHOD FOR PITCH CONTROL
  public void setShooterPitchAngleSetpoint(Angle angle) {
    shooterPitch.setMechanismPositionSetpoint(angle);
  }

  public Command setShooterPitchDutyCycle(double dutycycle) {
    return shooterPitch.set(dutycycle);
  }

  public Command shooterPitchSysId() {
    return shooterPitch.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4)); // Arbitrary Values
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterFlywheel.updateTelemetry();
    shooterPitch.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooterFlywheel.simIterate();
    shooterPitch.simIterate();
  }
}
