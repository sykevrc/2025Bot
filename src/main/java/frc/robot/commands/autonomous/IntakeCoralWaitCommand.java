package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class IntakeCoralWaitCommand extends Command{
    private EndEffectorSubsystem endEffectorSubsystem = RobotContainer.endEffectorSubsystem;
    private ElevatorSubsystem elevatorSubsystem = RobotContainer.elevatorSubsystem;
    private ArmSubsystem armSubsystem = RobotContainer.armSubsystem;

    public IntakeCoralWaitCommand() {
        addRequirements(endEffectorSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevatorSubsystem.setTargetPosition(Constants.ElevatorConstants.CoralHuman);
        armSubsystem.setDesiredState(ArmState.CoralHuman);
        endEffectorSubsystem.setDesiredState(EndEffectorState.IntakeCoralHumanElement);

        //System.out.println("IntakeCoralWaitCommand::execute() called");
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

        if(endEffectorSubsystem.hasCoral()) {
            System.out.println("IntakeCoralWaitCommand::execute() we have the coral so stopping");
        }

        // we don't return true until we have the coral
        return endEffectorSubsystem.hasCoral();
    }
}
