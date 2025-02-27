// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class FailsafeCoralCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final EndEffectorSubsystem endEffector = RobotContainer.endEffectorSubsystem;
  private final ElevatorSubsystem elevator = RobotContainer.elevatorSubsystem;
  private final ArmSubsystem arm = RobotContainer.armSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FailsafeCoralCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, endEffector, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setElevatorState(ElevatorSubsystem.ElevatorState.CoralL2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setArmPosition(Constants.CoralHuman);
    if (armSubsystem.getArmPosition() == Constants.CoralHuman) {
      elevator.setElevatorState(ElevatorSubsystem.ElevatorState.Start);
      endEffectorSubsystem.setEndEffectorState(EndEffectorSubsystem.EndEffectorState.Intake);
      armSubsystem.setArmState(ArmSubsystem.ArmState.ClearCoral);
    
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (armSubsystem.getArmState() == ArmSubsystem.ArmState.ClearCoral) {
      return true;
    }
    
    return false;
  }
}
