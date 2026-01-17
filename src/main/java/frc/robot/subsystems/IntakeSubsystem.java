// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


import com.revrobotics.ResetMode;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
 // private SparkMax intakeMotor = new SparkMax(1, MotorType.kBrushless);

 //Motors
 private TalonFX intakeMotor = new TalonFX(1, "rio");
 private TalonFX intakePivot = new TalonFX(2, "rio");

 //Requests 
 VelocityVoltage velocityRequest = new VelocityVoltage(0);
 PositionVoltage positionRequest = new PositionVoltage(0);



  public IntakeSubsystem() {
    motorConfigs();
  }

  public void motorConfigs(){
  TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration()
  .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Coast))
  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(120)).withStatorCurrentLimitEnable(true));

  TalonFXConfiguration intakePivotConfig = new TalonFXConfiguration()
  .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive).withNeutralMode(NeutralModeValue.Brake))
  .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(120)).withStatorCurrentLimitEnable(true));

  //There are two encoder tick to mechanism rotation conversion methods in TalonFX
  //RotorToSensorRatio is used when external encoder is used, or if encoder is geared relative to the motor
  //SensorToMechanismRatio directly connects to the built-in encoder on the talon, SparkMax equivalent to PositionConversionFactor and VelocityConversionFactor

  FeedbackConfigs feedback = new FeedbackConfigs();
  feedback.SensorToMechanismRatio = 5.0; //This value for 1 : 5 gear ratio, meaning that 5 spins in motor result in one mechanism rotation.
  //From Now on, all units are calculated in mechanism rotations.

  Slot0Configs pid = new Slot0Configs();
  pid.kP = 0.002;
  pid.kI = 0;
  pid.kD = 0;

  //Must have current limits
  CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
  limits.SupplyCurrentLimit = 40;
  limits.SupplyCurrentLimitEnable = true;

  //Soft Limit Configs
  var SoftLimitConfigs = new SoftwareLimitSwitchConfigs()
  .withForwardSoftLimitThreshold(100)
  .withForwardSoftLimitEnable(true)
  .withReverseSoftLimitThreshold(20)
  .withReverseSoftLimitEnable(true);

  intakePivot.getConfigurator().apply(intakePivotConfig);
  intakePivot.getConfigurator().apply(pid);
  intakePivot.getConfigurator().apply(limits);
  intakePivot.getConfigurator().apply(SoftLimitConfigs);
  intakePivot.getConfigurator().apply(feedback);
  
  intakeMotor.getConfigurator().apply(intakeMotorConfig);
  intakeMotor.getConfigurator().apply(limits);
  
  }

  public void configureBindings(){
   // SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
   // intakeMotorConfig.idleMode(com.revrobotics.spark.config.SparkMaxConfig.IdleMode.kBrake);
   
   // intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //FIX THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // public double getWristVelocity(){
  // return intakePivot.getVelocity(); //Mechanism Rotations per Second
  // }
  
  // public AngularPosition getWristPosition(){
  //   return intakePivot.getPosition().getValue(); //Mechanism Rotations
  // }
  

  public void intake(){
    //intakeMotor.set(0.5);
    intakeMotor.setControl(velocityRequest.withVelocity(15)); //RPS, Motor Velocity, not Wheel Velocity, Gear Reductions and Ratios must be converted
  }

  public void setIntakePosition(){
    intakePivot.setControl(positionRequest.withPosition(5)); //Target Rotations
  }

  public void resetWristPosition(){
    intakePivot.setPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
