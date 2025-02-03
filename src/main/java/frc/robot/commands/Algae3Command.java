package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;

public class Algae3Command extends Command {
    ElevatorSubsystem sliderSubsystem;
  ArmSubsystem armSubsystem;

  public Algae3Command() {
      this.sliderSubsystem = RobotContainer.sliderSubsystem;
      this.armSubsystem = RobotContainer.armSubsystem;
      
      addRequirements(sliderSubsystem);
      addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sliderSubsystem.setDesiredState(ElevatorState.AlgaeL3);
    armSubsystem.setDesiredState(ArmState.AlgaeL3);
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
