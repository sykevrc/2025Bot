package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;

public class Coral1Command extends Command {

    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;

    public Coral1Command() {
      this.elevatorSubsystem = RobotContainer.elevatorSubsystem;
      this.armSubsystem = RobotContainer.armSubsystem;
      
      addRequirements(elevatorSubsystem);
      addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setDesiredState(ElevatorState.CoralL1);
    armSubsystem.setDesiredState(ArmState.CoralL1);
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
