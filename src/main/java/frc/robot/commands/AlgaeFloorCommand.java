package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class AlgaeFloorCommand extends Command {

    private EndEffectorSubsystem endEffectorSubsystem;
    private ArmSubsystem armSubsystem;

    public AlgaeFloorCommand() {
        this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
        this.armSubsystem = RobotContainer.armSubsystem;

        addRequirements(endEffectorSubsystem);
        addRequirements(armSubsystem);
    }
    
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffectorSubsystem.setDesiredState(EndEffectorState.IntakeAlgaeFloor);
    armSubsystem.setDesiredState(ArmState.ArmFloor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
