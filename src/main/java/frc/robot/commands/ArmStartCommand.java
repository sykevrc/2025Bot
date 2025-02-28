package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.EndEffectorSubsystem.EndEffectorState;

public class ArmStartCommand extends Command {

    ArmSubsystem armSubsystem;
    EndEffectorSubsystem endEffectorSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    
    public ArmStartCommand() {
        this.armSubsystem = RobotContainer.armSubsystem;
        this.elevatorSubsystem = RobotContainer.elevatorSubsystem;
        this.endEffectorSubsystem = RobotContainer.endEffectorSubsystem;

        addRequirements(armSubsystem);
        addRequirements(endEffectorSubsystem);
        addRequirements(elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        armSubsystem.setDesiredState(ArmState.Start);
        //endEffectorSubsystem.setDesiredState(EndEffectorState.Stopped);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return true;

        //System.out.println("ArmStartCommand::isFinished() - position is: " + armSubsystem.getPosition());

        /*if(Math.abs(armSubsystem.getPosition() - Constants.ArmConstants.Start) <= 0.09) {
            System.out.println("done with ArmStartCommand");
            return true;
        }*/

        if(armSubsystem.atTargetPosition()) {
            System.out.println("ArmStartCommand::isFinished() - arm is at the target position so stopping");
            return true;
        }

        return false;
    }
    
}
