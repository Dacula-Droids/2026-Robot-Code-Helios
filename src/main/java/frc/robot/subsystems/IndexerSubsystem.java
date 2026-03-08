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
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorController;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */

  private static IndexerSubsystem INSTANCE;

   @SuppressWarnings("WeakerAccess")
  public static IndexerSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IndexerSubsystem();
    }
    return INSTANCE;
  }

  private TalonFX spinDexerMotor = new TalonFX(Constants.IndexerConstants.spinDexerMotorID, CANBus.roboRIO());
  

  // SMC Configs
  private SmartMotorControllerConfig smcSpinDexerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withTelemetry("SpinDexer", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(Constants.IndexerConstants.spinDexerGearRatio))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));


  // Smart Motor Controller Wrappers
  private SmartMotorController spinDexerSmartMotorController = new TalonFXWrapper(spinDexerMotor,
      DCMotor.getKrakenX60(1),
      smcSpinDexerConfig);


  // FlyWheel Configs for Each Motor
  private final FlyWheelConfig spinDexerFlywheelConfig = new FlyWheelConfig(spinDexerSmartMotorController)
      .withDiameter(Inches.of(2))
      .withMass(Pounds.of(0.3))
      .withUpperSoftLimit(RPM.of(2500))
      .withTelemetry("IndexerMech", TelemetryVerbosity.HIGH);


  // Mechanisms
  private FlyWheel spinDexer = new FlyWheel(spinDexerFlywheelConfig);


  // Commands
  public AngularVelocity getSpinDexerVelocity() {
    return spinDexer.getSpeed();
  }

  public Command setSpinDexerVelocity(AngularVelocity speed) {
    return spinDexer.run(speed);
  }

  public void setSpinDexerVelocitySetpoint(AngularVelocity speed) {
    spinDexer.setMechanismVelocitySetpoint(speed);
  }

  public Command setSpinDexerDutyCycle(double dutyCycle) {
    return spinDexer.set(dutyCycle);
  }

  public IndexerSubsystem() {

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    spinDexer.updateTelemetry();
  
  }

  @Override
  public void simulationPeriodic() {
    spinDexer.simIterate();
    
  }
}
