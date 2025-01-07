package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    public enum ArmState {
        CoralL4,
        CoralL3,
        CoralL2,
        CoralL1,
        AlgaeHuman,
        AlgaeL3,
        AlgaeL2,
        AlgaeL1,
        AlgaeShoot
    }

    private boolean isSim = false;
    private ArmState state = ArmState.AlgaeHuman;
    private double targetPosition = 0.0;
    private SparkMax motor = null;
    //private SparkClosedLoopController drivePID = motor.getClosedLoopController();
    private SparkClosedLoopController drivePID = null;

    private SparkMaxConfig config = new SparkMaxConfig();

    public ArmSubsystem() {

        if (Constants.enableArm) {

            if (RobotBase.isReal()) {
                isSim = false;
            } else {
                isSim = true;
            }

            //motor = new CANSparkMax(Constants.ArmConstants.motor_id, MotorType.kBrushless);
            motor = new SparkMax(Constants.ArmConstants.motor_id, MotorType.kBrushless);
            drivePID = motor.getClosedLoopController();

            if (isSim) {
                //REVPhysicsSim.getInstance().addSparkMax(motor, 2.6f, 5676);
                //SparkMaxSim maxSim = new SparkMaxSim(max, maxGearbox);
                DCMotor maxGearbox = DCMotor.getNEO(1);
                SparkMaxSim maxSim = new SparkMaxSim(motor, maxGearbox);
            }

            //motor.restoreFactoryDefaults();

            config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            //config.encoder
                //.positionConversionFactor(1000)
                //.velocityConversionFactor(1000);
            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(Constants.ArmConstants.P, Constants.ArmConstants.I, Constants.ArmConstants.D);
            config.signals.primaryEncoderPositionPeriodMs(5);

            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    public void setDesiredState(ArmState state) {
        this.state = state;

        switch(state) {
            case AlgaeHuman:
                targetPosition = 0.0;
                break;
            case AlgaeL1:
                targetPosition = 0.0;
                break;
            case AlgaeL2:
                targetPosition = 0.0;
                break;
            case AlgaeL3:
                targetPosition = 0.0;
                break;
            case AlgaeShoot:
                targetPosition = 0.0;
                break;
            case CoralL1:
                targetPosition = Constants.ArmConstants.CoralL1;
                break;
            case CoralL2:
                targetPosition = Constants.ArmConstants.CoralL2;
                break;
            case CoralL3:
                targetPosition = Constants.ArmConstants.CoralL3;
                break;
            case CoralL4:
                targetPosition = Constants.ArmConstants.CoralL4;
                break;
            default:
                targetPosition = 0.0;
                break;
            
        }

        drivePID.setReference(targetPosition, ControlType.kPosition);
    }

    @Override
	public void periodic() {
        drivePID.setReference(targetPosition, ControlType.kPosition);
    }
    
}
