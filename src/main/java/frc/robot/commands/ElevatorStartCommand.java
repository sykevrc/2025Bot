package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class ElevatorStartCommand extends Command {
    ElevatorSubsystem elevatorSubsystem;

    public ElevatorStartCommand() {
        this.elevatorSubsystem = RobotContainer.elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevatorSubsystem.setDesiredState(ElevatorState.Start);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if(elevatorSubsystem.atTargetPosition()) {
            System.out.println("ElevatorStartCommand::isFinished() - elevator is at the target position so stopping");
            return true;
        }

        return false;
    }
}
