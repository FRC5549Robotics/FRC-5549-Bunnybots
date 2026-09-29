package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class kidIntake extends SubsystemBase {

    //This is used to define The TalonFX as "intakeMotorLeft"
    private final TalonFX intakeMotorLeft;
    //private final TalonFX intakeMotorRight;

    // This is ised to define a SparkMax as "pivot motor"
    private final SparkMax pivotMotor;

    // Here we are defining this function as public so that other scripts may call it
    public kidIntake(){
        // We further define how the "pivot motor" is new and then we identify the motor with their brand, ID number, and motor type
        pivotMotor = new SparkMax(Constants.PIVOT_MOTOR_ID,MotorType.kBrushless);
       
        // Here we are setting the config of our motor "pivotConfig" to the SparkMaxConfig that is predetermined in the library
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        // Here we say that when it is not moving, then but it in "brake mode", more specifically idleMode
        pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(15);

        // Here we specifically call upon our motor, then go it it's configurations and set the config
        pivotMotor.configure(
            pivotConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters
        );

        // here we define our left motor and call upon the constants script in order to provide the ID of the motor in case it needs to be replaced
        intakeMotorLeft = new TalonFX(Constants.GROUND_INTAKE_ID, "lil clanker");
        //intakeMotorRight = new TalonFX(Constants.SECOND_GROUND_INTAKE_ID, "lil clanker");

        // Now we are configuring the intake instead of the previous pivot and we use the predetermined function once again
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        // In the intake config, we set the output of the motor when it is neutral mode to apply the brake
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set the current limits of the stator (electic/non moving motor), with the electricity/current it may intake to an int
        intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
        // Set the previous line to actually follow through but I don;t know where it is called/used
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // same thing as previous lines but rewrites over them?
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 35;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Setting the intake config, "openloop" means that it is controlled by a percentage instead of being adjusted
        intakeConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.08; //0.08s to reach max voltage

        // On the left intake motor be go to the configuration and apply the intake config to the motor
        intakeMotorLeft.getConfigurator().apply(intakeConfig);
    }

    // it is public because we want to be able to call it from other scripts, and it is void because it does not return anything, it just sets the motor to a certain speed
    public void pivotDown(){
        // if the motor position is less than the maximum then:
        if (pivotMotor.get() < Constants.PIVOT_DOWN_POSITION) {
            // We set the pivot motor to a specific speed, rotating it at 0.2 revolutions per second
             pivotMotor.set(0.2);
             intakeMotorLeft.set(0.4);
        }
    }

    // This is public beacause we want to call it from other scripts and void because it returns no value
    public void pivotStop(){
        // We set the pivot motor to 0, rotating it at 0 revolutions per second
        pivotMotor.set(0);
    }

    // This is public beacause we want to call it from other scripts and void because it returns no value
    public void pivotup(){
        // if the motor position is less than the maximum then:
        if (pivotMotor.get() > Constants.PIVOT_UP_POSITION) {
            // We set the pivot motor to a specific speed, rotating it at 0.2 revolutions per second
             pivotMotor.set(-0.2);
        }
    }
    public void intake() {
        intakeMotorLeft.set(0.4);
    }

}


