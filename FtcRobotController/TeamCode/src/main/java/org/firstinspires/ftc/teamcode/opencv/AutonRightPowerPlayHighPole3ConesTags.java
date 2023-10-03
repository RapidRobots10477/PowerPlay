package org.firstinspires.ftc.teamcode.opencv;

import static java.lang.Math.abs;
import java.util.ArrayList;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/**
 *
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required driving angle, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Drive Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired driving angle and the actual angle,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 */

@Autonomous(name="Auton Right ATags HighPole 3 Cones", group="AprilTags")

public class AutonRightPowerPlayHighPole3ConesTags extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    static final int LEFT   = 11;
    static final int MIDDLE = 13;
    static final int RIGHT  = 17;

    AprilTagDetection tagOfInterest = null;

    /**
     * Variables used for switching cameras.
     */
    private WebcamName webcam1, webcam2;
    private SwitchableCamera switchableCamera;



    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    //private TFObjectDetector tfod;

    /**
     * detectedSignal will be used for the Robot movement decision
     */
    private int detectedSignal = 13;
    
    private float signalConfidence = 0.0f;
    
    /* Declare OpMode members. */
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         rightFrontDrive  = null;
    private DcMotor         leftRearDrive    = null;
    private DcMotor         rightRearDrive   = null;
    //private DcMotor         armLiftDrive     = null;
    private DcMotor         armRightMotor    = null;
    private DcMotor         armLeftMotor     = null;

    private BNO055IMU       imu         = null;      // Control Hub IMU

    // create Claw Servo object and initialize to null
    //Servo clawServo = null;
    private Servo clampServoFront = null;
    private Servo clampServoRear = null;

    // hsvValues is an array that will hold the hue, saturation, and brightness value information
    private float hsvValues[] = {0F, 0F, 0F};

    private ColorSensor   colorSensorBottom    = null;
    private DistanceSensor distanceSensorFront = null;

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftFrontTarget    = 0;
    private int     rightFrontTarget   = 0;
    private int     leftRearTarget     = 0;
    private int     rightRearTarget    = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;   // GoBILDA 435 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.3;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    //static final double INITIAL_CLAW_POSITION = 0.61;
    static final double ARM_DOWN_POWER =  0.90;
    static final double ARM_UP_POWER   = -0.90;

//    static final double FRONT_CLAW_OPEN  = 0.45;
//    static final double FRONT_CLAW_CLOSE = 0.9;
//    static final double REAR_CLAW_OPEN   = 0.75;
//    static final double REAR_CLAW_CLOSE  = 0.9;
/*
    static final double FRONT_CLAW_OPEN  = 0.95;
    static final double FRONT_CLAW_CLOSE = 0.5;
    static final double REAR_CLAW_OPEN   = 0.85;
    static final double REAR_CLAW_CLOSE  = 0.6;
*/
    static final double FRONT_CLAW_OPEN  = 0.5;
    static final double FRONT_CLAW_CLOSE = 0.95;
    static final double REAR_CLAW_OPEN   = 0.6;
    static final double REAR_CLAW_CLOSE  = 0.95;

    static final int HIGH_POLE           = -2800;
    static final int MEDIUM_POLE         = -1900;
    static final int LOW_POLE            = -1600;

    static final int RELEASE_HIGH_POLE   = -2750;
    static final int RELEASE_MEDIUM_POLE = -1900;
    static final int RELEASE_LOW_POLE    = -975;
    
    static final int CONE_PICK_UP_1      = -400;
    static final int CONE_PICK_UP_2      = -450;

    @Override
    public void runOpMode() {
        
        //initialize OpenCV Camera
        initOpenCV();

        // config and initialize all motor config
        performMotorConfig();
        
        // while wait for the game to start (Display Gyro value while waiting)
        if (opModeInInit()) {
            telemetry.addData(">", "LeftFrontDrive Encoder = ", leftFrontDrive.getCurrentPosition());
            telemetry.addData(">", "LeftRearDrive Encoder = ", leftRearDrive.getCurrentPosition());
            telemetry.addData(">", "RightFrontDrive Encoder = ", rightFrontDrive.getCurrentPosition());
            telemetry.addData(">", "RightRearDrive Encoder = ", rightRearDrive.getCurrentPosition());
            telemetry.addData(">", "ArmRightMotor Encoder = ", armRightMotor.getCurrentPosition());
            telemetry.addData(">", "ArmLeftMotor Encoder = ", armLeftMotor.getCurrentPosition());
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }

        // Wait for the game to start
        closeClaw();
        
        //detect webcam signal
        detectedSignal = getDetectedSignalTag();
 
        // waiting for the play button press
        waitForStart();
 
        if (opModeIsActive()) {
            ElapsedTime startTimer = new ElapsedTime();
            startTimer.reset();

            //reset gyro angle
            resetHeading();
    
            sendTelemetry(true);
    
            closeClaw();
    
            //moveArmUp(-50);    // lift the cone up a inch just above the ground
    
            // Opening front claw to avoid stucking the signal cone between the claw and chasis
            //clampServoFront.setPosition(FRONT_CLAW_OPEN);
            
            moveArmUp(HIGH_POLE);   
            driveStraight(DRIVE_SPEED, 53.5, 0.0);
            holdHeading(TURN_SPEED,  0.0, 0.50);
            //driveStraight(DRIVE_SPEED, -15.5, 0.0);
            //holdHeading(TURN_SPEED,  0.0, 0.5);
    
            turnToHeading(TURN_SPEED, 45.0);
            holdHeading(TURN_SPEED,  45.0, 0.50);
            moveArmUp(HIGH_POLE);
            driveStraight(DRIVE_SPEED, 8, 45.0);
          
            moveArmDown(RELEASE_HIGH_POLE);
            holdHeading(TURN_SPEED,  45.0, 0.50);
            openClaw();
            //sendTelemetry(true);
            //sleep(20000);
            
            
            holdHeading(TURN_SPEED,  45.0, 0.25);
            driveStraight(DRIVE_SPEED, -12, 45.0);
            moveArmDown(CONE_PICK_UP_1);

            moveArmUp(CONE_PICK_UP_1); 
            
            turnToHeading(TURN_SPEED, 90);
            moveArmUp(CONE_PICK_UP_1);        
            driveStraight(DRIVE_SPEED, -24.0, 90);
            turnToHeading(TURN_SPEED, 90);
    
            //checkRobotPositionForConeStack();
            
            // First cone pick up from the cone stack
            moveArmUp(CONE_PICK_UP_1);
           
            driveStraight(DRIVE_SPEED, -7.0, 90);
            holdHeading(TURN_SPEED,  90.0, .25);
            closeClaw();
            holdHeading(TURN_SPEED,  90.0, 0.25); 
            moveArmUp(LOW_POLE);   
            
            holdHeading(TURN_SPEED,  90.0, .25);
    
            driveStraight(DRIVE_SPEED, 28, 90);
            turnToHeading(TURN_SPEED, 45.0);
            holdHeading(TURN_SPEED,  45.0, 0.25);
            driveStraight(DRIVE_SPEED, -11.0, 45);
            moveArmDown(RELEASE_LOW_POLE);
            openClaw();
           
            holdHeading(TURN_SPEED,  45, 0.25);
            driveStraight(DRIVE_SPEED, 11.0, 45);
            turnToHeading(TURN_SPEED, 90);


            // Second Cone pickup
       
            if (startTimer.time() >= 11.0) {
            
                moveArmDown(CONE_PICK_UP_2);        
                driveStraight(.60, -24.0, 90);
                turnToHeading(TURN_SPEED, 90);
        
                //checkRobotPositionForConeStack();
                
                // Second cone pick up from the cone stack
                moveArmUp(CONE_PICK_UP_2);
               
                driveStraight(DRIVE_SPEED, -7.0, 90);
                closeClaw();
                holdHeading(TURN_SPEED,  90.0, 0.25);    
                moveArmUp(LOW_POLE);   
                holdHeading(TURN_SPEED,  90.0, 0.25);    
    
                driveStraight(DRIVE_SPEED, 28, 90);
                turnToHeading(TURN_SPEED, 45.0);
                //holdHeading(TURN_SPEED,  -45.0, 0.5);
                driveStraight(DRIVE_SPEED, -11, 45);
                moveArmDown(RELEASE_LOW_POLE);  
    
                openClaw();
               
                holdHeading(TURN_SPEED,  45, 0.25);
                driveStraight(DRIVE_SPEED, 10.0, 45);
                turnToHeading(TURN_SPEED, 90);
            } 
            switch (detectedSignal) {
                case MIDDLE:
                    //Already in Position 2
                    //turnToHeading(TURN_SPEED, 0);
                    break;
                case LEFT:
                    //turnToHeading(TURN_SPEED, 90);
                    driveStraight(DRIVE_SPEED, 23.0, 90);
                    break;
                case RIGHT:
                    //turnToHeading(TURN_SPEED, 90);
                    driveStraight(DRIVE_SPEED, -23.0, 90);
                    break;
            }
            moveArmDown(0);
            closeClaw();
            
        }
        stopAll();
    }

    /**
     *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            leftRearTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            rightRearTarget = rightFrontDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);
            leftRearDrive.setTargetPosition(leftRearTarget);
            rightRearDrive.setTargetPosition(rightRearTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     *      * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     *      * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFrontDrive.setPower(leftSpeed);
        rightFrontDrive.setPower(rightSpeed);
        leftRearDrive.setPower(leftSpeed);
        rightRearDrive.setPower(rightSpeed);

        telemetry.addData("Left Speed>", "%5.2f", leftSpeed);
        telemetry.addData("Right Speed", "%5.2f", rightSpeed);
        telemetry.update();
        //sleep(500);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftFrontTarget,  rightFrontTarget);
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftRearTarget,  rightRearTarget);
            telemetry.addData("Actual Pos FL:FR",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
            telemetry.addData("Actual Pos RL:RR",  "%7d:%7d",      leftRearDrive.getCurrentPosition(), rightRearDrive.getCurrentPosition());
            telemetry.addData("detected Signal >", detectedSignal);
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
    /**
     * This method will return true/false whether the robot is in the proximity
     * @param distance Proximity distance
     * @return
     */
    public boolean isObjectCloser(int distance) {
        boolean isCloser = distanceSensorFront.getDistance(DistanceUnit.CM) < distance;
        return isCloser;
    }

    /**
     * This method will raise the arm for a certain length with one go
     * @param holdPosition
     */
    private void moveArmUp(int holdPosition) {

        try {
            new Thread(new Runnable() {
                public void run() {

                    armRightMotor.setTargetPosition(holdPosition);
                    armLeftMotor.setTargetPosition(holdPosition);
                    while (armLeftMotor.getCurrentPosition() > holdPosition) {
                        armLeftMotor.setPower(ARM_UP_POWER);
                        armRightMotor.setPower(ARM_UP_POWER);
                        telemetry.addData("RightArmPosition>", armRightMotor.getCurrentPosition());
                        telemetry.addData("LeftArmPosition>", armLeftMotor.getCurrentPosition());
                        telemetry.update();
                    }
            
                    armLeftMotor.setPower(0);
                    armRightMotor.setPower(0);
                    armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }).start();
        }catch(RuntimeException re) {
            //eat this runtime excption
        }

    }

    /**
     *
     * @param holdPosition
     */
    private void moveArmDown(int holdPosition) {
        while (holdPosition > armLeftMotor.getCurrentPosition()) {
            //armLiftDrive.setPower(ARM_DOWN_POWER);
            armLeftMotor.setPower(ARM_DOWN_POWER);
            armRightMotor.setPower(ARM_DOWN_POWER);
            //telemetry.addData("ArmPosition>", armLiftDrive.getCurrentPosition());
            telemetry.addData("RightArmPosition>", armRightMotor.getCurrentPosition());
            telemetry.addData("LeftArmPosition>", armLeftMotor.getCurrentPosition());
            telemetry.update();
        }
        //armLiftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armLeftMotor.setPower(0);
                    armRightMotor.setPower(0);
                    armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /**
     * 
     */ 
    private void moveArmDownAsync(int holdPosition) {
        try {
            new Thread(new Runnable() {
                public void run() {
                    while (holdPosition > armLeftMotor.getCurrentPosition()) {
                        //armLiftDrive.setPower(ARM_DOWN_POWER);
                        armLeftMotor.setPower(ARM_DOWN_POWER);
                        armRightMotor.setPower(ARM_DOWN_POWER);
                        //telemetry.addData("ArmPosition>", armLiftDrive.getCurrentPosition());
                        telemetry.addData("RightArmPosition>", armRightMotor.getCurrentPosition());
                        telemetry.addData("LeftArmPosition>", armLeftMotor.getCurrentPosition());
                        telemetry.update();
                    }
                    //armLiftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            }).start();
        }catch(RuntimeException re) {
            //eat this runtime excption
        }
    }
    /**
     * Open Claw
     */
    private void openClaw() {
        clampServoFront.setPosition(FRONT_CLAW_OPEN);
        clampServoRear.setPosition(REAR_CLAW_OPEN);
    }

    /**
     * Close Claw
     */
    private void closeClaw(){
        clampServoFront.setPosition(FRONT_CLAW_CLOSE);
        clampServoRear.setPosition(REAR_CLAW_CLOSE);
    }
    

    /**
     * Perform all motor and gyro config and initialization
     */
    private void performMotorConfig(){

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftRearDrive   = hardwareMap.get(DcMotor.class, "left_rear");
        rightRearDrive  = hardwareMap.get(DcMotor.class, "right_rear");
        //armLiftDrive    = hardwareMap.get(DcMotor.class, "arm_lift");
        armRightMotor   = hardwareMap.get(DcMotor.class, "right_arm");
        armLeftMotor    = hardwareMap.get(DcMotor.class, "left_arm");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);
        
        armLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        armRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        clampServoFront = hardwareMap.get(Servo.class, "front_clamp");
        //clampServoFront.setPosition(INITIAL_CLAW_POSITION);

        clampServoRear = hardwareMap.get(Servo.class, "rear_clamp");
        //clampServoRear.setPosition(INITIAL_CLAW_POSITION);


        //colorSensorBottom = hardwareMap.get(ColorSensor.class, "center_color");
        //distanceSensorFront = hardwareMap.get(DistanceSensor.class, "left_front_distance");

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //armLiftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //armLiftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //armLiftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
    }
    /**
     * Align the robot to the blue/red strip at the center 
     * 
     */
    private boolean checkRobotPositionForConeStack() {
        boolean isAlignedForConeStack = false;
        double leftSpeed = 0.30;
        ElapsedTime elapsedTime = new ElapsedTime();
        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensorBottom.red() * 8, colorSensorBottom.green() * 8, colorSensorBottom.blue() * 8, hsvValues);
        elapsedTime.reset();
        while (colorSensorBottom.alpha() < 12 && elapsedTime.time() < 1.0) {    // color_sensor.argb()
            // Strafe the robot in the direction
            leftFrontDrive.setPower(leftSpeed);
            rightFrontDrive.setPower(rightSpeed);
            leftRearDrive.setPower(-leftSpeed);
            rightRearDrive.setPower(-rightSpeed);
        }
        return isAlignedForConeStack;
    }

    /**
     *
     *
     */

    private void initOpenCV(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

    }

    /**
     *
     *
     */
    private int getDetectedSignalTag(){

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagToTelemetry(tagOfInterest);
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        //telemetry.update();
        return (tagOfInterest != null? tagOfInterest.id : MIDDLE);
    }
    /**
     *
     */

    private void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    /**
     * 
     */
    public void stopAll(){
        moveRobot(0, 0);
        armRightMotor.setPower(0);
        armLeftMotor.setPower(0);
        
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
