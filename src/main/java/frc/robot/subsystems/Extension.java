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

// Define our class/name and give it the same variables/properties as the SubsystemBase
public class Extension extends SubsystemBase{

    // Set the PID to not be enabled and make it only ever be a True or False
    private boolean PIDEnabled = false; //default to PID being off

    // We are making a variable called extensionPID that implements a PID control loop and set it equal to a defined controller with specifc values
    private final PIDController extensionPID = new PIDController(0.1, 0.0, 0.004); // kP, kI, kD
    // We are making a double with the name kS and set it equal to 0.12
    private static final double kS = 0.12;

    // Set the extensionSetpoint equal to 0 for future declerations in the future
    private double extensionSetpoint = 0;

    {
        // This calls extensionPID, then sets the tolerance to 10.2, so basically if the robot it within 1.2 units, it is considered at its goal
        extensionPID.setTolerance(1.2); //1 motor rotation of tolerance = 1/9 bottom pulley tolerance
    }

    //private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    // make a variable called homingRequest and DutyCycleOut is used to control TalonFX motors
    // we will set it to reverse/go down I think, slowly, more precisely at -8.5%
    private final DutyCycleOut homingRequest = new DutyCycleOut(-0.085);

    // Define Extension Motor and classify it as TalonFX
    TalonFX ExtensionMotor;
    // Define ExtensionMotorConfig and classify it a config 
    TalonFXConfiguration ExtensionMotorConfig;

    // Mkae a function call Extension()
    public Extension(){
        // Set the Extension motor variable to a new irl motor with the motor ID defined in the file "constants" and name it "lil clanker"
        ExtensionMotor = new TalonFX(Constants.EXTENSION_MOTOR_ID, "lil clanker");
        // We will then set the motor config equal to the premade config in their library
        ExtensionMotorConfig = new TalonFXConfiguration();

        // Set the extension motors configuration to be limited to 50 units for the Stator
        ExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = 50;
        // Make sure the the configurations are actually enabled
        ExtensionMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // Set the extension motors configuration to be limited to 50 units for the Stator
        ExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = 35; 
        // Make sure the the configurations are actually enabled
        ExtensionMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Go to the configurations and set the motors output when in "neutral" equal to being in brake
        ExtensionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // In the configuarations, set the standard controls and inside that...
        // Show how the robot compensates for gravity and make it so it understands it is a rotating arm rather than a linear elevator
        ExtensionMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // In the configurations, when it is a closed loop/active I think, more specifically while we ramp between modes
        // set the amount of time it akes to get from one state to another to be 0.05 seconds
        ExtensionMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

        // In the configurations, in Motion Magic, set the movement velocity to be 4
        ExtensionMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 4; // cap on velocity
        // In the configurations, in motionMagic, set the acceleration to be 8
        ExtensionMotorConfig.MotionMagic.MotionMagicAcceleration = 8; // cap on acceleration

        // In the configurations, for the motor output when inverted, set that value to make it spin clockwise and negative to be counter-clockwise
        ExtensionMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // In our extension motor, use the function getConfigurator, and apply our config we just made
        ExtensionMotor.getConfigurator().apply(ExtensionMotorConfig);

    }

    // Make a function to extend
    public void extend() {
        //set the extensionSetpoint to be 21.5
        extensionSetpoint = 21.5;
    }

    // Make a function to retract
    public void retract() {
        // Set the extensionSetpoint to be equal to 0
        extensionSetpoint = 0;
        // When we retract it, print in our terminal that it is being set to point 0
        System.out.println("hopper retracted");
    }

    // Make a function that makes a marker when it is called
    public boolean atTarget() {
        // When this funtion is called, extensionPID will set a point at that location for future use
        return extensionPID.atSetpoint();
    }

    //Make a function called "extensionDownSlow"
    public void extensionDownSlow() {
        // We will set the extensionSetpoint to be 0
        extensionSetpoint = 0;
        // Call the motor, and set that control to be equal to the homing request
        ExtensionMotor.setControl(homingRequest);
    }

    // Why is the commented out?

    //public void stop() {
        //ExtensionMotor.setControl(new NeutralOut());
        //System.out.println("extension motor stop() ran");
        //PIDEnabled = true;
    //}

    // make a function called "zeroEncoder"
    public void zeroEncoder() {
        // Call the extensionMotor and set the speed to "NeutralOut"
        ExtensionMotor.setControl(new NeutralOut());
        // Call the extensionMotor and set the position equal to 0
        ExtensionMotor.setPosition(0);
        //Enable the PID
        PIDEnabled = true;
        // Print in the terminal that the PID is active
        System.out.println("PID enabled = true");
    }

    // make a function that answers True or False and name it "atBottom"
    public boolean atBottom() {
        //boolean velocitySlow = Math.abs(ExtensionMotor.getVelocity().getValueAsDouble()) < 0.05;
        //SmartDashboard.putBoolean("velocitySlow", velocitySlow);

        //boolean currentSpiked = ExtensionMotor.getStatorCurrent().getValueAsDouble() > 25.0;
        //SmartDashboard.putNumber("current", ExtensionMotor.getStatorCurrent().getValueAsDouble());

        //return velocitySlow && currentSpiked;

        //This function always returns true... why??
        return true; //just assume its at the bottom and we put it all the way down
    }

    // make a new function that returns a boolean and name it "atBottomTeleop"
    public boolean atBottomTeleop() {
        // make a boolean variable named "velocitySlow" and set it True if the general speed of the motor's less than 0.05
        boolean velocitySlow = Math.abs(ExtensionMotor.getVelocity().getValueAsDouble()) < 0.05;
        //SmartDashboard.putBoolean("velocitySlow", velocitySlow);

        // make a variable that is a boolean named "currentSpiked" and make it equal to True if the amount of current going through the stator is more than 25
        boolean currentSpiked = ExtensionMotor.getStatorCurrent().getValueAsDouble() > 25.0;
        //SmartDashboard.putNumber("current", ExtensionMotor.getStatorCurrent().getValueAsDouble());

        // This function will return True if both variables are true
        return velocitySlow && currentSpiked;

        // In general this function returns true if the motor is moving slow and has a lot of current going through it
    }

    // Make a function named "getPosition"
    public double getPosition() {
        // Will return a double that is the position of where the motor is
        return ExtensionMotor.getPosition().getValueAsDouble();
    }

    //This will override any previous mentions of "periodic()"
    @Override
    // Make a function that doesnt return anything called "periodic()"
    public void periodic() {
        // Sets the curront position as a double equal to the position of the motor
        double currentPos = ExtensionMotor.getPosition().getValueAsDouble();
        // Is this not just the same as getPosition()??


        //SmartDashboard.putNumber("extension Target", extensionSetpoint);
        //SmartDashboard.putNumber("extension Position", currentPos);
        //SmartDashboard.putBoolean("extension PID enabled", PIDEnabled);

        // If the PID is False then stop this function
        if (!PIDEnabled) return;

        // Set the output as a double and make it equal to the calculation of the PID with the current position vs where the extension should be
        double output = extensionPID.calculate(currentPos, extensionSetpoint);

        //SmartDashboard.putNumber("Hood Output", output);

        // if the absolute value of the output is less than 0.001:
        if (Math.abs(output) > 0.001) {
            // Add 1 or -1 times kS to the output
            output += Math.signum(output) * kS;
        }

        // Set the output to be equal to itself unless it is less than -0.4 or greather than 0.4
        output = MathUtil.clamp(output, -0.4,0.4);

        // If the PID says atTarget() is True then:
        if (atTarget()) {
            //if (extensionSetpoint == 20 && currentPos < 19.5) {
                //ExtensionMotor.setControl(new DutyCycleOut(0.1).withEnableFOC(true));
            //} else {
                // set the control of the motor equal to the brake/stop it
                ExtensionMotor.setControl(new NeutralOut());
            //}
            // if the PID says atTarget() is false:
        } else {
            // Set the extension of the motor to the outputa and enable the FOC
            ExtensionMotor.setControl(new DutyCycleOut(output).withEnableFOC(true)); //move the hood
        }
    }
}