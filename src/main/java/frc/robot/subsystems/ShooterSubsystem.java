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
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  private static ShooterSubsystem INSTANCE;

   @SuppressWarnings("WeakerAccess")
  public static ShooterSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ShooterSubsystem();
    }
    return INSTANCE;
  }

  // Motors
  private TalonFX shooterFlywheelMotor = new TalonFX(Constants.ShooterConstants.shooterFlywheelMotorID,
      CANBus.roboRIO());
  private TalonFX shooterFollowerFlywheelMotor = new TalonFX(Constants.ShooterConstants.shooterFlywheelFollowerMotorID, CANBus.roboRIO());
  private TalonFX shooterPitchMotor = new TalonFX(Constants.ShooterConstants.shooterPitchMotorID, CANBus.roboRIO());
  private CANcoder pitchEncoder = new CANcoder(Constants.ShooterConstants.shooterThroughboreEncoderID);

  // SMC Configs
  private SmartMotorControllerConfig shooterFlywheelSmcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(0.18746, 0, 0.00, DegreesPerSecond.of(10000000), DegreesPerSecondPerSecond.of(100000)) //0.435, 0, 0.00
      .withSimClosedLoopController(1, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withFeedforward(new SimpleMotorFeedforward(0.19, 0.1125, 0)) //0.5, 0.149, 0.032819 // 2; 0.19, 0.116279069767
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0)) 
      .withTelemetry("Shooter Flywheel Motor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(Constants.ShooterConstants.shooterFlywheelGearRatio)) // 1:1 Gear Ratio
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withClosedLoopRampRate(Seconds.of(0.01))
      .withStatorCurrentLimit(Amps.of(80));

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
      .withMass(Pounds.of(4.1))
      .withSoftLimit(RPM.of(0),RPM.of(100000))
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

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,                    // default quasistatic ramp = 1 V/s
            Units.Volts.of(7),       // dynamic step voltage (lower = safer, prevents brownout)
            null,                    // default timeout = 10 s
            (state) -> SignalLogger.writeString("state", state.toString()) // required for SysId
        ),
        new SysIdRoutine.Mechanism(
            (Voltage volts) -> shooterFlywheelMotor.setControl(voltageRequest.withOutput(volts.in(Units.Volts))),
            null,                    // logging callback = null (Phoenix Signal Logger handles everything)
            this                     // subsystem reference for logging
        )
    );
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  public void setVoltage(double volts) {
    shooterFlywheelMotor.setControl(voltageRequest.withOutput(volts));
  }

  

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

  public void setFlywheelVelocityMPS(LinearVelocity mps){
    this.setShooterFlywheelVelocitySetpoint(RPM.of(mps.in(MetersPerSecond) * 187.978279242 * 2.35));
  }
  

  public Command sysId() { 
  
  // Our Static test will run the arm up and down with 7v.
  // Our Dynamic test will run the arm up and down going from 0v to 7v inreasing at 2v per second.
  // The test will last 4 seconds at most.
  return shooterPitchSysId();
  }

  public Command setShooterFlywheelDutyCycle(double dutyCycle) {
    return shooterFlywheel.set(dutyCycle);
  }

  public void setShooterVoltage(double voltage){
    shooterFlywheelMotor.setVoltage(voltage);
  }

  


  // DO NOT USE UNLESS YOU KNOW WHAT THIS DOES!
  public Command setPitchAngle(Angle angle) {
    return shooterPitch.run(angle);
  }



// ... inside ShooterSubsystem ...

public Command setFullSpeed() {
    return this.run(() -> {
        // 1.0 represents 100% output (full battery voltage)
        shooterFlywheelMotor.setControl(new DutyCycleOut(1));
    });
}

  // USE THIS FOR CLOSED LOOP CONTROL AS MAIN METHOD FOR PITCH CONTROL
  public void setShooterPitchAngleSetpoint(Angle angle) {
    shooterPitch.setMechanismPositionSetpoint(angle);
  }

  public Command setShooterPitchDutyCycle(double dutycycle) {
    return shooterPitch.set(dutycycle);
  }

  public Command shooterPitchSysId() {
    return shooterFlywheel.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(8)); // Arbitrary Values
  }

  public void setPitchVoltage(double voltage){
    shooterPitchMotor.setVoltage(voltage);
  }

  public void zeroPitchEncoder(){
    shooterPitchMotor.setPosition(0.25);
  }

  ShooterNN shooterNN;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    try {
      //Pass in the name of the folder (must be in deploy directory) containing you layer Weights and Biases along with the specifications.json
      shooterNN = new ShooterNN("team8865fixed65degv1"); 
    } catch (Exception e){}

    shooterFollowerFlywheelMotor.setControl(new Follower(Constants.ShooterConstants.shooterFlywheelMotorID, MotorAlignmentValue.Opposed));
  }

   // Should probably put these in a contansts file
  Transform2d robotToExitTransform = new Transform2d(-0.1524,0, new Rotation2d());
  Distance FIELD_X = Units.Meters.of(16.513048); // Length of full field in the x
  Distance FIELD_Y = Units.Meters.of(8.042656); // Length of full field in the y
  Pose2d redAllianceOrigin = new Pose2d(FIELD_X.magnitude(), FIELD_Y.magnitude(), Rotation2d.fromDegrees(180));
  Distance WALL_MARGIN = Units.Meters.of(0.1);
  Distance HUB_MARGIN = Units.Meters.of(0.1);

  // Assumes that robotPose2d is given relative to blue alliance origin. If this is not the case, handle it.
  public Optional<double[]> getShotParams(Pose2d robotPose2d, ChassisSpeeds robotVelocity){
    var alliance = DriverStation.getAlliance();

    //Convert robotPose2d and robotvelocity relative to red alllince if on red
    // if (DriverStation.getAlliance().isPresent() && alliance.get() == Alliance.Red) {
    //   robotPose2d = robotPose2d.relativeTo(redAllianceOrigin);
    //   robotVelocity = new ChassisSpeeds(-robotVelocity.vxMetersPerSecond, -robotVelocity.vyMetersPerSecond, robotVelocity.omegaRadiansPerSecond);
    // }

    // Checks if the pos is within the shooting area minus the wall and hub margins. 
    // This is highly recommended as it makes sure that you won't get bad outputs from the NN because of inputs outside or near the edge of the data it was trained on.
    // if (!shooterNN.isPosValid(robotPose2d, robotToExitTransform, WALL_MARGIN, HUB_MARGIN)) return Optional.empty();

    //for ShooterType.VarPitch_VarSpeed, returns: [heading, launch_angle, launch_speed]
    //for ShooterType.VarPitch_FixedSpeed, returns: [heading, launch_angle]
    //for ShooterType.FixedPitch_VarSpeed, returns: [heading, launch_speed]
    double[] shotParams = shooterNN.getShotParams(robotPose2d, robotVelocity, robotToExitTransform);
    double req_heading = shotParams[0];


    // If you don't have a Yaw controlled turret and are using your swerve drive to set your heading, 
    // you should reject large heading changes as the latency will likely be too large.
    // If you do this, I recommend having your robot already aiming somwhere near the hub before calling this method as it minimizes latency
    // if (Math.abs(req_heading - robotPose2d.getRotation().getRadians()) > Math.PI/2) return Optional.empty();
    
    return Optional.of(shotParams);
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
