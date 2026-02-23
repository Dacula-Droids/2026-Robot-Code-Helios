package frc.robot.Utils;
import static edu.wpi.first.units.Units.Meters;

import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim implements IntakeIO {
    private final IntakeSimulation intakeSimulation;
    public IntakeIOSim(AbstractDriveTrainSimulation driveTrainSimulation) {
        // Here, create the intake simulation with respect to the intake on your real robot
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
        // Specify the type of game pieces that the intake can collect
        "Fuel",
        // Specify the drivetrain to which this intake is attached
        driveTrainSimulation,
        // Width of the intake
        Meters.of(0.67945), // 26.75 inches
        // The extension length of the intake beyond the robot's frame (when activated)
        Meters.of(0.35052), // 13.8 inches
        // The intake is mounted on the back side of the chassis
        IntakeSimulation.IntakeSide.BACK,
        // The intake can hold up to 1 note
        10);
    }
}