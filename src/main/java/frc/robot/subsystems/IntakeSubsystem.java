// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils.Preset;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static IntakeSubsystem INSTANCE;

  public static IntakeSubsystem getInstance(){
    if(INSTANCE==null) {
      INSTANCE = new IntakeSubsystem();
    }
    return INSTANCE;
 
  }

  private SparkMax intakeMotor = new SparkMax(Constants.ArmConstants.kIntakeMotorId,MotorType.kBrushless);
  private SparkMax endEffector = new SparkMax(Constants.ArmConstants.kEndEffectorMotorId,MotorType.kBrushless);

  public IntakeSubsystem() {
    MotorConfigs();

  }

  public void MotorConfigs(){
    SparkMaxConfig endEffectorConfig = new SparkMaxConfig();
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

    endEffectorConfig.inverted(false).idleMode(IdleMode.kBrake);
    endEffectorConfig.closedLoopRampRate(0.3);
    endEffectorConfig.closedLoop.feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder).pid(0.00002,0,0).outputRange(-1,1);
    

   
    endEffectorConfig.encoder.positionConversionFactor(14.4);
    endEffectorConfig.encoder.velocityConversionFactor(0.24);

    intakeMotorConfig.inverted(false).idleMode(IdleMode.kCoast);

    //Bounds
    endEffectorConfig.softLimit.forwardSoftLimit(Constants.ArmConstants.forwardSoftLimit);
    endEffectorConfig.softLimit.reverseSoftLimit(Constants.ArmConstants.reverseSoftLimit);
    endEffectorConfig.softLimit.forwardSoftLimitEnabled(true);
    endEffectorConfig.softLimit.reverseSoftLimitEnabled(true);

    

    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    endEffector.configure(endEffectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  public void moveIntake(Preset preset){
    endEffector.getClosedLoopController().setSetpoint(preset.position, ControlType.kPosition);
  }

  public double getWristPosition(){
    return endEffector.getEncoder().getPosition();
  }

  public void runIntake(){
    intakeMotor.set(Constants.ArmConstants.intakeMotorSpeed);
  }

  public void joystickMoveIntake(double motorSpeed){
  endEffector.getClosedLoopController().setSetpoint(motorSpeed, ControlType.kVelocity);
  }
  
  public void stopIntake(){
    intakeMotor.set(0);
  }

  public void runOutake(){
    intakeMotor.set(Constants.ArmConstants.outakeMotorSpeed);
  }


  public boolean isIntakeAtPosition(Preset preset){
  double target  = preset.position;
  double current = endEffector.getEncoder().getPosition();
  double tolerance = Constants.ArmConstants.endEffectorTolerance;

  return Math.abs(target - current) <= tolerance;

  }

  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
