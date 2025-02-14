package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class AlgaeProcessorCommand extends Command {
  ElevatorSubsystem elevatorSubsystem;
  ArmSubsystem armSubsystem;
  EndEffectorSubsystem endEffectorSubsystem;

  public AlgaeProcessorCommand() {
      this.elevatorSubsystem = RobotContainer.elevatorSubsystem;
      this.armSubsystem = RobotContainer.armSubsystem;
      this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
      
      addRequirements(elevatorSubsystem);
      addRequirements(armSubsystem);
      addRequirements(endEffectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setDesiredState(ElevatorState.AlgaeHuman);
    armSubsystem.setDesiredState(ArmState.AlgaeHuman);
    endEffectorSubsystem.setDesiredState(EndEffectorState.IntakeAlgaeFloor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
