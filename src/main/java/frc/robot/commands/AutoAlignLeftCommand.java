package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.tools.Limelight;

public class AutoAlignLeftCommand extends Command {

    private DriveSubsystem driveSubsystem;
    private Limelight limelight;
    private boolean finished = false;

    public AutoAlignLeftCommand() {
        this.driveSubsystem = RobotContainer.driveSubsystem;
        this.limelight = RobotContainer.limelight;
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(limelight.hasTarget()) {
            double aprilTagLocation = LimelightHelpers.getTX(Constants.LimelightConstants.name);

            /*System.out.println("aprilTagLocation: " + aprilTagLocation + " location offset: " + aprilTagLocation
                + " left: " + (-Constants.DriveConstants.kAutoAlignOffset + Constants.DriveConstants.kAutoAlignTolerance)
                + " right: " + (-Constants.DriveConstants.kAutoAlignOffset - Constants.DriveConstants.kAutoAlignTolerance)
            );*/

            if(
                (aprilTagLocation ) < (-Constants.DriveConstants.kAutoAlignOffset + Constants.DriveConstants.kAutoAlignTolerance)
                && (aprilTagLocation) > (-Constants.DriveConstants.kAutoAlignOffset - Constants.DriveConstants.kAutoAlignTolerance)
            ) {
               // System.out.println("we are here");
                driveSubsystem.driveRobotRelative(0.0, 0.0, 0.0);
                //finished = true;
            } else if(
                (aprilTagLocation) < (-Constants.DriveConstants.kAutoAlignOffset + Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                //System.out.println("move left");
                //driveSubsystem.drive(0.0, Constants.DriveConstants.kAutoAlignSpeed, 0.0);
                driveSubsystem.driveRobotRelative(0.0, Constants.DriveConstants.kAutoAlignSpeed, 0.0);
            } else if (
                (aprilTagLocation) > (-Constants.DriveConstants.kAutoAlignOffset - Constants.DriveConstants.kAutoAlignTolerance)
            ) {
                //System.out.println("move right");
                //driveSubsystem.drive(0.0, -Constants.DriveConstants.kAutoAlignSpeed, 0);
                driveSubsystem.driveRobotRelative(0.0, -Constants.DriveConstants.kAutoAlignSpeed, 0.0);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
}
