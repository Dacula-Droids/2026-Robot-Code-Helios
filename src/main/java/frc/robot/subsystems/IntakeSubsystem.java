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

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.IntakeState;
import frc.robot.Utils.Preset;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem INSTANCE;
  private TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID, "rio");
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

  private SmartMotorControllerConfig pivotSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new ArmFeedforward(0, 0, 0))
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      .withTelemetry("Intake Pivot Motor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(Constants.IntakeConstants.intakePivotGearRatio)) // 12:1 Gear Ratio
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25)); // PID Controller, Max Velocity, Max Acceleration;

  private TalonFXWrapper pivotSmartMotorController = new TalonFXWrapper(pivotMotor, DCMotor.getKrakenX60(1),
      pivotSmcConfig);

  private PivotConfig pivotConfig = new PivotConfig(pivotSmartMotorController)
      .withSoftLimits(Preset.Intake.position, Preset.Stowed.position)
      .withHardLimit(Preset.Intake.position, Preset.Stowed.position)
      .withStartingPosition(Preset.Stowed.position)
      .withTelemetry("Intake Pivot", TelemetryVerbosity.HIGH);

  private Pivot intakePivot = new Pivot(pivotConfig);

  // Set angle of Intake, but command and Intake does not stop
  public Command setAngle(Angle angle) {
    return intakePivot.run(angle);
  }

  // public Command setAngleAndStop(Angle angle){
  // return intakePivot.runTo(angle);
  // }

  // Closed Loop controller for Intake
  public void setPivotSetpoint(Angle angle) {
    intakePivot.setMechanismPositionSetpoint(angle);
  }

  // dutycycle, incase open loop needed during testing
  public Command setPivotDutyCycle(double dutycycle) {
    return intakePivot.set(dutycycle);
  }

  public Command sysId() {
    return intakePivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  private SmartMotorControllerConfig rollerSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(1, 0, 0)
      .withSimClosedLoopController(1, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withTelemetry("Intake Roller Motor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(Constants.IntakeConstants.intakeRollerGearRatio)) // 1:1 Gear Ratio
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private TalonFXWrapper rollerSmartMotorController = new TalonFXWrapper(rollerMotor, DCMotor.getKrakenX60(1),
      rollerSmcConfig);

  private final FlyWheelConfig intakeRollerConfig = new FlyWheelConfig(rollerSmartMotorController)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(0.21))
      .withUpperSoftLimit(RPM.of(1000))
      .withTelemetry("Roller Mechanism", TelemetryVerbosity.HIGH);

  private FlyWheel intakeRoller = new FlyWheel(intakeRollerConfig);

  public Command setRollerVelocity(AngularVelocity speed) {
    return intakeRoller.run(speed);
  }

  public void setVelocitySetpoint(AngularVelocity speed) {
    intakeRoller.setMechanismVelocitySetpoint(speed);
  }

  public Command setRollerDutyCycle(double dutycycle) {
    return intakeRoller.set(dutycycle);
  }

  // Important Command to Set "State"
  public Command setState(IntakeState state) {
    return this.runOnce(() -> {
      setPivotSetpoint(state.PivotAngle);

      switch (state) {
        case INTAKING, OUTTAKING, HOLDING ->
          setVelocitySetpoint(state.RollerSpeed);

        default ->
          setVelocitySetpoint(RPM.of(0));
      }
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakePivot.updateTelemetry();
    intakeRoller.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intakePivot.simIterate();
    intakeRoller.simIterate();
  }
}
