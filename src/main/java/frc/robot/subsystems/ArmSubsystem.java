package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.EnumSet;
import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

    public enum ArmState {
        Start,
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
    private SparkFlex motor = null;
    private SparkFlexSim motorSim = null;
    private SparkClosedLoopController pid = null;
    private SparkFlexConfig config = new SparkFlexConfig();

    private double p = Constants.ArmConstants.P;
    private double i = Constants.ArmConstants.I;
    private double d = Constants.ArmConstants.D;

    public ArmSubsystem() {

        if (Constants.kEnableArm) {

            if (RobotBase.isReal()) {
                isSim = false;
            } else {
                isSim = true;
            }

            motor = new SparkFlex(Constants.ArmConstants.motor_id, MotorType.kBrushless);

		    if(isSim) {
			    motorSim = new SparkFlexSim(motor, DCMotor.getNeoVortex(1));
		    }

            setConfig();

		    pid = motor.getClosedLoopController();

            if (Constants.kEnableDebugArm) {

                Shuffleboard.getTab("Arm")
                    .addDouble("Position", this::getPosition)
                    .withWidget(BuiltInWidgets.kTextView);

                Shuffleboard.getTab("Arm")
                    .addDouble("Target", this::getPosition)
                    .withWidget(BuiltInWidgets.kTextView);

                SmartDashboard.putData(this);
                Shuffleboard.getTab("Arm").add(this);

            }
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

        //drivePID.setReference(targetPosition, ControlType.kPosition);
    }

    @Override
	public void periodic() {

        if (Constants.kEnableArm) {
            pid.setReference(targetPosition, ControlType.kPosition);
            //pid.setReference(targetPosition, SparkBase.ControlType.kMAXMotionPositionControl);

            if (isSim) {
                motorSim.iterate(
                        // 0.1,
                        // desiredState.speedMetersPerSecond,
                        motor.getOutputCurrent(),
                        RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
                        0.02);
            }
        }
    }
    
    private void setConfig() {
        config = new SparkFlexConfig();

        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        //config.encoder
        //    .positionConversionFactor(25)
        //    .velocityConversionFactor(25);
        config.closedLoop
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
		    .pid(
			    p, 
                i, 
                d
			);

        // Set MAXMotion parameters
        config.closedLoop.maxMotion
            .maxVelocity(6784)
            .maxAcceleration(1)
            //.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
            .allowedClosedLoopError(.1);

        config.signals.primaryEncoderPositionPeriodMs(5);

        motor.configure(
		    config, 
			ResetMode.kResetSafeParameters, 
			PersistMode.kPersistParameters
		);
    }

    public double getPosition() {

        if(isSim) {
            return motorSim.getRelativeEncoderSim().getPosition();
        }

        return motor.getEncoder().getPosition();
    }

    public double getTargetPosition() {

        return this.targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double getP() {
        return this.p;
    }

    public void setP(double p) {
        this.p = p;
        setConfig();
    }

    public double getI() {
        return this.i;
    }

    public void setI(double i) {
        this.i = i;
        setConfig();
    }

    public double getD() {
        return this.d;
    }

    public void setD(double d) {
        this.d = d;
        setConfig();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotPreferences");
        builder.addDoubleProperty("D", this::getD, this::setD);
        builder.addDoubleProperty("I", this::getI, this::setI);
        builder.addDoubleProperty("P", this::getP, this::setP);
        builder.addDoubleProperty("Target", this::getTargetPosition, this::setTargetPosition);
        builder.addDoubleProperty("Position", this::getPosition,null);
    }
}
