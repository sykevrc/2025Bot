package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class StartCommand extends Command {
    ElevatorSubsystem elevatorSubsystem;
    ArmSubsystem armSubsystem;
    EndEffectorSubsystem endEffectorSubsystem;

  public StartCommand() {
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
    elevatorSubsystem.setDesiredState(ElevatorState.Start);
    armSubsystem.setDesiredState(ArmState.Start);
    endEffectorSubsystem.setDesiredState(EndEffectorState.Stopped);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // need to check for elevator subsystem as well
    if(armSubsystem.atTargetPosition()) {
      return true;  
    }

    return false;
    
  }
}
