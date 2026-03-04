// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */
  private static KickerSubsystem INSTANCE;

  @SuppressWarnings("WeakerAccess")
  public static KickerSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new KickerSubsystem();
    }
    return INSTANCE;
  }

  private TalonFX kickerMotor = new TalonFX(Constants.IndexerConstants.kickerMotorID, CANBus.roboRIO());

  private SmartMotorControllerConfig smcKickerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withTelemetry("Kicker", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(Constants.IndexerConstants.kickerGearRatio))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController kickerSmartMotorController = new TalonFXWrapper(kickerMotor, DCMotor.getKrakenX60(1),
      smcKickerConfig);

  private final FlyWheelConfig kickerFlywheelConfig = new FlyWheelConfig(kickerSmartMotorController)
      .withDiameter(Inches.of(2))
      .withMass(Pounds.of(0.3))
      .withUpperSoftLimit(RPM.of(2500))
      .withTelemetry("IndexerMech", TelemetryVerbosity.HIGH);

  private FlyWheel kicker = new FlyWheel(kickerFlywheelConfig);


  public AngularVelocity getKickerVelocity() {
    return kicker.getSpeed();
  }

  public Command setKickerVelocity(AngularVelocity speed) {
    return kicker.run(speed);
  }

  public void setKickerVelocitySetpoint(AngularVelocity speed) {
    kicker.setMechanismVelocitySetpoint(speed);
  }

  public Command setKickerDutyCycle(double dutyCycle) {
    return kicker.set(dutyCycle);
  }

  public KickerSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    kicker.updateTelemetry();
  }
}
