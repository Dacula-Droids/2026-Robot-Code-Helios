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

import swervelib.simulation.ironmaple.simulation.IntakeSimulation;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SimSwerveDrivetrain;
import com.fasterxml.jackson.annotation.JsonGetter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.IntakeState;
import frc.robot.Utils.IntakePreset;
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
import lombok.Getter;
import yams.motorcontrollers.SmartMotorController;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem INSTANCE;
  private TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.intakeMotorID, CANBus.roboRIO());
  private TalonFX intakePivotMotor = new TalonFX(Constants.IntakeConstants.pivotMotorID, CANBus.roboRIO());

  @Getter
  private final IntakeSimulation intakeSim;

  @SuppressWarnings("WeakerAccess")
  public static IntakeSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
  }

  SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    if (RobotBase.isSimulation()) {
      intakeSim = IntakeSimulation.OverTheBumperIntake("Fuel",
          swerveSubsystem.getSwerveDrive().getMapleSimDrive().get(), Constants.IntakeConstants.intakeWidth,
          Constants.IntakeConstants.intakeExtensionLength, IntakeSimulation.IntakeSide.FRONT,
          Constants.IntakeConstants.maxGamePieceCapacity);

      intakeSim.register();
    } else {
      intakeSim = null;
    }
  }

  private SmartMotorControllerConfig pivotSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(170, 0, 1, DegreesPerSecond.of(375), DegreesPerSecondPerSecond.of(250))
      .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new ArmFeedforward(0.3, 0.25, 2)) //0.3 ks, 0.25 kg, 2 kv
      .withSimFeedforward(new ArmFeedforward(0, 0, 0))
      .withTelemetry("Intake Pivot Motor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(Constants.IntakeConstants.intakePivotGearRatio)) // 12:1 Gear Ratio
      .withMotorInverted(true)
      .withIdleMode(MotorMode.BRAKE)
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25)); // PID Controller, Max Velocity, Max Acceleration;

  private SmartMotorController pivotSmartMotorController = new TalonFXWrapper(intakePivotMotor, DCMotor.getKrakenX60(1),
      pivotSmcConfig);

  private PivotConfig intakePivotConfig = new PivotConfig(pivotSmartMotorController)
      .withSoftLimits(Constants.IntakeConstants.lowerIntakeSoftLimit, Constants.IntakeConstants.upperIntakeSoftLimit)
      .withHardLimit(IntakePreset.Intake.position, IntakePreset.Stowed.position)
      .withStartingPosition(IntakePreset.Stowed.position)
      .withTelemetry("Intake Pivot", TelemetryVerbosity.HIGH)
      .withStartingPosition(IntakePreset.Stowed.position)
      .withMOI(Constants.IntakeConstants.intakeCenterOfMassFromPivot, Constants.IntakeConstants.intakeMass);

  private Pivot intakePivot = new Pivot(intakePivotConfig);

  // Set angle of Intake, but command and Intake does not stop
  public Command setIntakeAngle(Angle angle) {
    return intakePivot.run(angle);
  }

  public void setPivotVoltage(double voltage) {
    intakePivotMotor.setVoltage(voltage);
  }

  // public Command setAngleAndStop(Angle angle){
  // return intakePivot.runTo(angle);
  // }

  // Closed Loop controller for Intake
  public void setIntakePivotSetpoint(Angle angle) {
    intakePivot.setMechanismPositionSetpoint(angle);
  }

  // dutycycle, incase open loop needed during testing
  public Command setIntakePivotDutyCycle(double dutycycle) {
    return intakePivot.set(dutycycle);
  }

  public Command sysId() {
    return intakePivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));
  }

  private SmartMotorControllerConfig rollerSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.002772, 0, 0.001, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(1, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new SimpleMotorFeedforward(0, 0.12, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withTelemetry("Intake Roller Motor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(Constants.IntakeConstants.intakePivotGearRatio))) // 1:1
                                                                                                                      // Gear
                                                                                                                      // Ratio
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController rollerSmartMotorController = new TalonFXWrapper(rollerMotor, DCMotor.getKrakenX60(1),
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

  public void setRollerVelocitySetpoint(AngularVelocity speed) {
    intakeRoller.setMechanismVelocitySetpoint(speed);
  }

  public Command setRollerDutyCycle(double dutycycle) {
    return intakeRoller.set(dutycycle);
  }

  // Important Command to Set "State"
  public Command setState(IntakeState state) {
    return this.runOnce(() -> {

      setIntakePivotSetpoint(state.PivotAngle);
      setRollerVelocitySetpoint(state.RollerSpeed);

      if (RobotBase.isSimulation() && intakeSim != null) {

        switch (state) {
          case INTAKING -> intakeSim.startIntake();
          case OUTTAKING -> intakeSim.stopIntake();
          case HOLDING, STOWED -> intakeSim.stopIntake();
        }
      }
    });
  }

  public Command intakePivotTest() {
    return this.runOnce(() -> {
      setIntakePivotSetpoint(IntakePreset.Test.position);
    });
  }

  public Command setIntakeAngle() {
    return intakePivot.run(IntakePreset.Intake.position);
  }

  public Command setIntakeZero() {
    return intakePivot.run(IntakePreset.Stowed.position);
  }

  public Command intakePivotZero() {
    return this.runOnce(() -> {
      setIntakePivotSetpoint(IntakePreset.Stowed.position);
    });
  }

  public int getGamePieceCount() {
    if (RobotBase.isSimulation() && intakeSim != null) {
      return intakeSim.getGamePiecesAmount();
    } else {
      return 0;
    }
  }

  public void zeroPivotEncoder() {
    intakePivotMotor.setPosition(0.25); // Mechanism Rotations

  }

  public boolean hasGamePiece() {
    return getGamePieceCount() > 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakePivot.updateTelemetry();
    intakeRoller.updateTelemetry();
    SmartDashboard.putNumber("intake Pivot Angle Degrees", intakePivotMotor.getPosition().getValue().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    intakePivot.simIterate();
    intakeRoller.simIterate();
  }
}
