package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class AlgaeFloorCommand extends Command {

    private EndEffectorSubsystem endEffectorSubsystem;
    private ArmSubsystem armSubsystem;
    private ElevatorSubsystem elevatorSubsystem;

    public AlgaeFloorCommand() {
        this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
        this.armSubsystem = RobotContainer.armSubsystem;
        this.elevatorSubsystem = RobotContainer.elevatorSubsystem;

        addRequirements(endEffectorSubsystem);
        addRequirements(armSubsystem);
        addRequirements(elevatorSubsystem);
    }
    
    // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setDesiredState(ElevatorState.AlgaeFloor);   
    armSubsystem.setDesiredState(ArmState.ArmFloor);

    /*if(endEffectorSubsystem.hasCoral()) {
      endEffectorSubsystem.setDesiredState(EndEffectorState.EjectAlgaeFloor);
    } else {
      endEffectorSubsystem.setDesiredState(EndEffectorState.IntakeAlgaeFloor);
    }*/

    endEffectorSubsystem.setDesiredState(EndEffectorState.IntakeAlgaeFloor);

    //System.out.println("AlgaeFloorCommand::execute() called");
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
