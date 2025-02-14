package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class IntakeNoWait extends Command{
    private EndEffectorSubsystem endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
    private ElevatorSubsystem elevatorSubsystem = RobotContainer.elevatorSubsystem;

    public IntakeNoWait() {
        addRequirements(endEffectorSubsystem);
        addRequirements(elevatorSubsystem);
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //elevatorSubsystem.setTargetPosition(Constants.ElevatorConstants.CoralHuman);
        endEffectorSubsystem.setDesiredState(EndEffectorState.IntakeCoralHumanElement);
        //System.out.println("IntakeNoWait::execute() called");
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
