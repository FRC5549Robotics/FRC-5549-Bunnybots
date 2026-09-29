package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class kidextension extends SubsystemBase { //practice subsystem by connor and vanessa we are sorry if this breaks the robot
    private boolean PIDEnabled = false; //default to PID being off

    private final PIDController extensionPID = new PIDController(0.1, 0.0, 0.004); // kP, kI, kD
    private static final double kS = 0.12;

    private double extensionSetpoint = 0;

    {
        extensionPID.setTolerance(1.2); //1 motor rotation of tolerance = 1/9 bottom pulley tolerance
    }

    //private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    // Here we set our homingRequest to be equal to -0.085 and our "DutyCycleOut" means that it becomes an output
    private final DutyCycleOut homingRequest = new DutyCycleOut(-0.085);

    TalonFX ExtensionMotor;
    TalonFXConfiguration ExtensionMotorConfig;

    public kidextension(){
        ExtensionMotor = new TalonFX(Constants.EXTENSION_MOTOR_ID, "lil CLANKER");
        ExtensionMotorConfig = new TalonFXConfiguration();

        ExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = 50;
        ExtensionMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        ExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = 35; 
        ExtensionMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        ExtensionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        ExtensionMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        ExtensionMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

        ExtensionMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 4; // cap on velocity
        ExtensionMotorConfig.MotionMagic.MotionMagicAcceleration = 8; // cap on acceleration

        ExtensionMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        ExtensionMotor.getConfigurator().apply(ExtensionMotorConfig);
    }
    // Here we are making another function named goUp()
    public void goUp(){
        // Here we set the extensionSetpoint to 20, but idk what it does
        extensionSetpoint = 20;
    }
    // Another function decleration named goDown()
    public void goDown(){
        // We set the extensionSetpoint to 0/our base at the bottom
        extensionSetpoint = 0;
    }

       public boolean PossiblyAtTarget() {
        return extensionPID.atSetpoint();
    }
    // Here we do the same thing as goDown()
    public void goDownButSlow(){
        // We set extensionSetpoint to 0
        extensionSetpoint = 0;
        // We go to a method of our ExtensionMotor to set the control speed to our homing request/set speed
        ExtensionMotor.setControl(homingRequest);
    }
    // making another function to set it to the middle
    public void goMiddle(){
        // Set the extensionSetpoint to 10, which I believe is the middle and I dont know hwy they want that
        extensionSetpoint = 10;
    }
    // We are making this a public function called bossAroundHopperwopper()
    public void bossAroundHopperwopper(){
        // Call the extension motor and set the speed of the motor to nothing?
        ExtensionMotor.setControl (new NeutralOut());
        // We set the position of the motor to be at the bottom/0
        ExtensionMotor.setPosition(0);
        // I don't know what PID means but we enable it here
        PIDEnabled = true;
        // Print in our terminal that we successfully completed this action
        System.out.println("GG Fuking EZZZZZ");
    }
    // Make a command saying that it is or is not at the bottom of its range so that we don't run other down commands
    public boolean leBottoming(){
        return true;
    }
    // make a function called "controllerSayBottom"
    public boolean controllerSaysBottom(){
                // We make a variable called variableSlow say and we make it a boolean
                // more specifically we get the absolute value of our extension motors velocity, typecast it as a double and return true if it is less than 0.05
                boolean velocitySlow = Math.abs(ExtensionMotor.getVelocity().getValueAsDouble()) < 0.05;
            // We make a variable called "currentSpiked" and call the extension motor
            // We set that value as a double and return true if it is less 25
             boolean currentSpiked = ExtensionMotor.getStatorCurrent().getValueAsDouble() > 25.0;
                    // We will then return True based on whether it returns both as True
                     return velocitySlow && currentSpiked;

    }
    // This is a function named "whereAmI"
    public double whereAmI(){
    // This will return the position/extension of motor from base position as a double
    return ExtensionMotor.getPosition().getValueAsDouble();
    }

    // This is used above a function to redefine a name to a specific function
    // This is used when you have multiple names for things and you want to override what you previously had
    @Override

    // Make a function named "periodic" and make sure that it does not return anything
    public void periodic() {
        // set the current position as a double equal to the extension motors position as a double
        double currentPos = ExtensionMotor.getPosition().getValueAsDouble();
        // If the PID is not enabled then we want to return it although if it is enabled then return nothing
        if (!PIDEnabled) return;
        // Make a variable called output that called our extension PID and calculate it sa the current position
        double output = extensionPID.calculate(currentPos, currentPos);

        if (Math.abs(output) > 0.001 ) {
            output += Math.signum(output) * kS;
        }

        output = MathUtil.clamp(output, -0.4, 0.4);

        if (PossiblyAtTarget()){
            ExtensionMotor.setControl(new NeutralOut());
        }
        else {
            ExtensionMotor.setControl(new DutyCycleOut(output).withEnableFOC(true));
        }
    }
}
