// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootingCommand extends ParallelCommandGroup {
  /** Creates a new ShootingCommand. */
  public ShootingCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem,
      KickerSubsystem kickerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
        shooterSubsystem.setShooterFlywheelDutyCycle(0.75), //.run(() -> shooterSubsystem.setShooterFlywheelVelocitySetpoint(RPM.of(8 * 187.978279242 * 1.425))),
        indexerSubsystem.setSpinDexerDutyCycle(1),
        kickerSubsystem.setKickerDutyCycle(1));
  }

}
