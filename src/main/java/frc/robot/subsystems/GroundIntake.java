package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {

    private final SparkMax pivotMotor;
    private final TalonFX intakeMotorLeft;
    //private final TalonFX intakeMotorRight;
    private final DutyCycleEncoder pivotAbsEncoder;

    private final Extension m_Extension;

    // your existing setpoints (continuous rotations style)
    private final double PIVOT_UP_POSITION   = 145;
    private final double PIVOT_DOWN_POSITION = 268;

    // Make a variable and set it to False called "holdingAtTop"
    private boolean holdingAtTop = false;

    // Set voltageRequest as a data type and set it equal to 0 voltage and with field oriented control enabled
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

    //right follows left
    //private final Follower intake_Right_Follower = new Follower(Constants.GROUND_INTAKE_ID, MotorAlignmentValue.Opposed);

    // Make a public function that is an extension of extension, meaning the whole script's variables and methods may be used
    public GroundIntake(Extension extension) {
        // We define m_extension to be equal to the extension that we call withing our function
        this.m_Extension = extension;

        // Set our pivot motor to be a new sparkmax and give it the motor ID that we defined in our constants file, then specify the type of motor  it is
        pivotMotor = new SparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        // We are making a varible called pivotAbsEncoder and set it equal to the detector of where the pivot is located
        pivotAbsEncoder = new DutyCycleEncoder(
            // Reference our constants file and call a variable
            Constants.PIVOT_ABS_ENC_DIO,
            // Then inside that variable, define fullRange and expectedZero
            360.0,   // full range = 360 degrees per rotation
            0.0      // zero position (can change later)

        );

        // Set a variable named pivotConfig that is equal to a defult SparkMaxConfig
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        // Call our pivotConfig and set the idleMode brake to be limited to 15 units of current
        pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(15);

        // Within the pivotMotor we want to configure...
        pivotMotor.configure(
            pivotConfig,
            // Set the reset mode to be within safe parameters s in the future we can safely reset the robot
            com.revrobotics.ResetMode.kResetSafeParameters,
            // We also make the persist mode have to persist with the constraints given
            com.revrobotics.PersistMode.kPersistParameters
        );

        // We set the intakeMotorLeft and cast it as a TalonFX, we also go to constants and find the ID of the specific motot
        intakeMotorLeft = new TalonFX(Constants.GROUND_INTAKE_ID, "lil clanker");
        //intakeMotorRight = new TalonFX(Constants.SECOND_GROUND_INTAKE_ID, "lil clanker");

        // make a variable named intakeConfig that is cast as TalonFX configuration, we also give it its on defult set of TalonFXConfigurations
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        // Call our intakeConfig and go to the motors output and further define the neautral mode to be automatically a brake
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Change the intake config's current limits and make that stators current limit to be 60
        intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
        // Change the intake configs current limits so that the limits are actually enabled
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // Change the intake config's current limits and make that stators current limit to be 35
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 35;
        // Change the intake configs current limits so that the limits are actually enabled
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Go into the intakeConfigs ramp settings and change it so that it goes from 0% to 100% within 0.08 seconds
        intakeConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.08; //0.08s to reach max voltage

        // We call the intakeLeft and apply the configurations that we just defined to the motor
        intakeMotorLeft.getConfigurator().apply(intakeConfig);
        //intakeMotorRight.getConfigurator().apply(intakeConfig);

        //intakeMotorRight.setControl(intake_Right_Follower);
         
    }

    // name a function that returns True or False to be "extensionSafe()"
    private boolean extensionSafe() {
        // Will will return True if the oppers position is greater than 10
        return m_Extension.getPosition() > 10;
    }

    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled
    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled

    // make a function named getPivotPosition() that returns a value as a double
    public double getPivotPosition() {
        //System.out.println(pivotTargetRotations);
        // Returns absolute position in DEGREES (0–360)
        // This function will return the pivot motors location from it's encoder
        return pivotAbsEncoder.get();
    }

// Make a function named setPivotUp() that returns nothing
public void setPivotUp() {
    // Here we call the extension file through the variable we previously set up and pull it up
    m_Extension.extend();
    // We will print in our termilnal "run pls" when the opper goes up
    System.out.println("run pls");

    // We will set the posiiton at the pivots position
    double position = getPivotPosition();

    // Bottom is ~156°, top is ~40°
    // Move upward until we reach ~40°
    if (position > 40) {
        // If the extension is safe/opper's value is above 10
        if (extensionSafe()) {
            //We will make the motor go up
            pivotMotor.set(0.6);
        }
    // if the position is higher than 40 then...
    } else {
        // Set the pivot motor to not move
        pivotMotor.set(0.0);
        // Make the input motor stop as well
        intakeMotorLeft.set(0.0);
        // Because this activates when it is higher than 40, state that it is at the top
        holdingAtTop = true;
        // Print in our terminal that it is all the way up
        System.out.println("All the way up");
    }
}

    // make a function that returns nothing and is called "setPivotDown"
     public void setPivotDown() {
        // Set the holdingAtTop variable to be false because we are about to move it
        holdingAtTop = false;
        //intakeMotorLeft.setControl(voltageRequest.withOutput(10)); //6
        // Print in our terminal that we called the down command
        System.out.println("down called");
        // make another variable that defines the pivots position
        var pivot_pos = getPivotPosition();
        // We will print in our terminal the pivot motors position
        System.out.println(pivot_pos);
        // If the pivit motors position is less than 140, then...
        if (getPivotPosition() < 140) {
            // If the pivots position is greater than or equal to 40 then...
            if (getPivotPosition() >= 40) { //if intake is too far back, extend hopper before deploying intake
                // instanciate the extension/opper and extend it to the top
                m_Extension.extend();
                // Print within our terminal "go go go"
                System.out.println("go go go ");
                // If the extension is safe, then..
                if (extensionSafe()) {
                    // set the pivot motor to go down
                    pivotMotor.set(-0.4);
                    // Print within our terminal that the extension is safe
                    System.out.println("extension safe");
                    // Make the intake motor on the left start running
                    intakeMotorLeft.set(.4);
                }
            // If the pivot position is less than 40...
            } else {
                // Make the pivot motor go down
                pivotMotor.set(-0.4); //if its already out enough, don't worry about the extension
                // Print in our terminal that we have set the pivot motor to go down
                System.out.println("pivotmotorset-0.4");
            }
        }
        // If the position is lower that the bottom we have set then...
         else {
            // Make the pivot motor stop
            pivotMotor.set(0.0);
            // Call the intake motor on the left (only one) and set the control to our voltage request which we previously defined that has FOC
            // Then we change our voltage request to be 10 instead of 0
            intakeMotorLeft.setControl(voltageRequest.withOutput(10));
            // System.out.println("we chill");
        }
    }

// Encoder positions:
// Bottom = ~90
// Top    = ~342

// We will make a function called "setPivotUpFully()"
public void setPivotUpFully() {
    // Print in our terminal that we are trying to go up
    System.out.println("trying to go up");
    // extend our opper so that it doesnt get caught
    m_Extension.extend();

    // If the pivots position is greater than 40/ lower than our highest needed point then...
    if (getPivotPosition() > 40) {

        // If our extension is within safe parapeters which we previously set:
        if (extensionSafe()) {
                    // make our pivot motor go up
                    pivotMotor.set(-0.4);
                    // In our terminal we will print "extension safe"
                    System.out.println("extension safe");
                    // make the intake motor spin
                    intakeMotorLeft.set(.4);
                }
    // If the pivots position is less than 40
    } else {
        // make the pivot motor go lower
        pivotMotor.set(0.4);
        // Mkae the intake stop
        intakeMotorLeft.set(0.0);
        // set the variable holding at top to be True
        holdingAtTop = true;
    }
}

// Make a function called "setPivotDownFast"
public void setPivotDownFast() {

    // Change our variable that we are holding at the top to be False
    holdingAtTop = false;

    // If our pivot sposiiton is less than 150...
    if (getPivotPosition() < 150) {
        // Extend the opper to the top
        m_Extension.extend();
        // If the extension is greater than 10:
        if (extensionSafe()) {
                // Move the pivot down
                pivotMotor.set(-0.4);
            }

         else {

            // Already out far enough; don't worry about extension
            pivotMotor.set(-0.4);
        
        } 
    }
    else {

        pivotMotor.set(0.0);
    }
}



    public void retractForExtension() {
        double pos = getPivotPosition();

        if (pos < 190 && pos > 170) { //if pivot is too far up but not all the way back
            pivotMotor.set(0.4); //fully retract the intake
        }
    }

    // public void zeroPivotEncoder() {
    //     pivot
    // }

    public void shooting() {
        m_Extension.extend();
        holdingAtTop = false;
        double pos = getPivotPosition();
        
        //System.out.println(pos);
        if (pos < 140) {
            if (extensionSafe()) {
                pivotMotor.set(-0.4);
            }
        } else if (pos > 33) {
            if (extensionSafe()) {
                pivotMotor.set(0.5);
            }
        }

              // if (pos < 100){
        //     if(extensionSafe()){
        //         pivotMotor.set(-0.4);
        //     }
        // }
        //  else if (pos > 33) {
        //     if (extensionSafe()) {
        //         pivotMotor.set(0.4);
        //     }
        // }
        // else if (pos < 160) {
        //     if (extensionSafe()) {
        //         pivotMotor.set(0.4);
        //     }
        // }

        // if(pos > 97){
        //     if(extensionSafe()){
        //         pivotMotor.set(.4);
        //     }
        // }
        // else if (pos< 97){
        //     pivotMotor.set(-.4);
        // }
    }
    

    public void IntakeReverse() {
        intakeMotorLeft.set(-0.3);
    }

    public void IntakeOn(){
        intakeMotorLeft.set(.5);
    }

    public void off() {
        //System.out.println("pivotDisabled");
        intakeMotorLeft.set(0.0);
        pivotMotor.set(0.0);
        holdingAtTop = false;
    }

    @Override
    public void periodic() {
        // ALWAYS push raw encoder debug so you can see if it's alive
        double raw = pivotAbsEncoder.get();

        //SmartDashboard.putNumber("GI DIO Channel", Constants.PIVOT_ABS_ENC_DIO);
        SmartDashboard.putBoolean("GI Enc Connected", pivotAbsEncoder.isConnected());
        SmartDashboard.putNumber("GI Enc Degrees", raw);

        if (holdingAtTop) {
            pivotMotor.set(0.1);
        }
        
        //SmartDashboard.putNumber("extension position", m_Extension.getPosition());
    }
}
