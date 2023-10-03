package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TeleOp DrivePowerPlayGame Two Arms")
@Disabled
public class DrivePowerPlayGameTwoArms extends OpMode {

    // create motor objects and initialize to null
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;
    //private DcMotor armLiftMotor = null;
    private DcMotor armRightMotor = null;
    private DcMotor armLeftMotor = null;
    private BNO055IMU imu = null;      // Control Hub IMU

    private PIDController  pidRotate, pidDrive;

    // create Claw Servo object and initialize to null
    //Servo clawServo = null;
    private Servo clampServoFront = null;
    private Servo clampServoRear = null;

    private ColorSensor   colorSensorBottom    = null;
    private DistanceSensor distanceSensorFront = null;
    private DigitalChannel leftLimitSwitch     = null;
    private DigitalChannel rightLimitSwitch    = null;

    boolean speed_lmt_on;
    boolean servoCheck;

    double driveSpeed, currentSpeedCap, forward, strafe;
    double rotate;

    double arm_lift_speed;

    static final double SPEED_CAP_MIN    = 0.5;
    static final double SPEED_CAP_MAX    = 0.75;
    
    static final double ARM_DOWN_POWER   = 1.0;
    static final double ARM_UP_POWER     = -1.0;

    static final double FRONT_CLAW_OPEN  = 0.5;
    static final double FRONT_CLAW_CLOSE = 0.9;
    static final double REAR_CLAW_OPEN   = 0.75;
    static final double REAR_CLAW_CLOSE  = 0.9;
/**    
    static final double FRONT_CLAW_CLOSE = 0.75;
    static final double FRONT_CLAW_OPEN  = 0.1;
    static final double REAR_CLAW_CLOSE  = 0.8;
    static final double REAR_CLAW_OPEN   = 0.1;
*/
    static final int HIGH_POLE           = -4000;
    static final int MEDIUM_POLE         = -3000;
    static final int LOW_POLE            = -2000;

    static final double ROTATE_SPEED = 0.75;
            // Max rotate speed to limit turn rate
        // Max rotate speed to limit turn ate
    static final double TURN_SPEED = 0.4;           // Max Turn speed to limit turn rate
    static final double HEADING_THRESHOLD = 1.0;    // the target before moving to next step.
    static final double P_TURN_GAIN = 0.02;         // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;        // Larger is more responsive, but also less stable

    static final int TASK_ZERO       = 0;
    static final int TASK_ONE        = 1;
    static final int TASK_TWO        = 2;
    static final int TASK_THREE      = 3;

    private double targetHeading = 0;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;

    public int taskNumber = 0;

    boolean rotate_check, rotate_check_press;

    boolean speedToggleCurrent, speedCapTogglePrev  = false;

    int arm_left_motor_pos, arm_right_motor_pos;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    @Override
    public void init() {
        /**
         *
         * Map all hardware ports
         */

        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front");
        leftBackMotor = hardwareMap.dcMotor.get("left_rear");
        rightBackMotor = hardwareMap.dcMotor.get("right_rear");
        //armLiftMotor = hardwareMap.dcMotor.get("arm_lift");
        armRightMotor = hardwareMap.dcMotor.get("right_arm");
        armLeftMotor = hardwareMap.dcMotor.get("left_arm");

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        pidRotate = new PIDController(.003, .00003, 0);

        clampServoFront = hardwareMap.get(Servo.class, "front_clamp");
        clampServoRear = hardwareMap.get(Servo.class, "rear_clamp");
        leftLimitSwitch = hardwareMap.get(DigitalChannel.class, "left_limit");
        leftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightLimitSwitch = hardwareMap.get(DigitalChannel.class, "right_limit");
        rightLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        
        //armLiftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLeftMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRightMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        currentSpeedCap = SPEED_CAP_MIN;
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        
        armLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        armRightMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        switch (taskNumber)
        {
            case TASK_ZERO:
                /* Read control button status for base vehicle movement */
                forward = gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
                rotate = gamepad1.right_stick_x;

                arm_lift_speed = gamepad2.left_stick_y;
                
                checkSpeedToggle();

                forward = performSpeedCap(forward);
                strafe = performSpeedCap(strafe);

                moveRobot(forward, strafe, true);

                /* Apply power to Macanum wheels to rotate the robot */
                if (rotate > 0) {
                    if (rotate > ROTATE_SPEED) {
                        rotate = ROTATE_SPEED;
                    }
                    moveRobot(0, rotate);
                }
                if (rotate < 0) {
                    if (rotate < -ROTATE_SPEED) {
                        rotate = -ROTATE_SPEED;
                    }
                    moveRobot(0, rotate);
                }
                taskNumber = TASK_ONE;
            break;

            case TASK_ONE:
                limitArmDownMove();

                if (arm_lift_speed < ARM_UP_POWER) {
                        arm_lift_speed = ARM_UP_POWER;
                } else if (arm_lift_speed > ARM_DOWN_POWER) {
                        arm_lift_speed = ARM_DOWN_POWER;
                }
        

                if (gamepad2.y) {
                    moveArmUp(HIGH_POLE);
                } else if (gamepad2.b) {
                    moveArmUp(MEDIUM_POLE);
                } else if (gamepad2.a) {
                    moveArmUp(LOW_POLE);
                }
                else {
                    armLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armLeftMotor.setPower(arm_lift_speed);
                    armRightMotor.setPower(arm_lift_speed);
                }
        
                // gamepad2.left_bumper  for closing both claws
                if (gamepad2.left_bumper) {
                    closeClaw();
                }
                // gamepad2.right_bumper  for opening both claws
                if (gamepad2.right_bumper) {
                    openClaw();
                }
                taskNumber = TASK_ZERO;
            break;
            default:
                taskNumber = TASK_ZERO;
            break;
        }
        sendTelemetry(true);
    }

    /**
     *
     * @param drive
     * @param turn
     */
    public void moveRobot(double drive, double turn) {
        moveRobot(drive, turn, false);
    }

    /**
     * * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     * @param strafe activate strafe rather than turn
     */
    public void moveRobot(double drive, double turn, boolean strafe) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }
        if (strafe && turn != 0.0) {
            leftFrontMotor.setPower(leftSpeed);
            rightFrontMotor.setPower(rightSpeed);
            leftBackMotor.setPower(-leftSpeed);
            rightBackMotor.setPower(-rightSpeed);
        }else {
            leftFrontMotor.setPower(leftSpeed);
            rightFrontMotor.setPower(rightSpeed);
            leftBackMotor.setPower(leftSpeed);
            rightBackMotor.setPower(rightSpeed);
        }

    }

    /**
     * This method will raise the arm for a certain length with one go
     * and this will execute in a non-blocking way
     */
    private void moveArmUp(int holdPosition) {
        armLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLeftMotor.setTargetPosition(holdPosition);
        armRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRightMotor.setTargetPosition(holdPosition);
        armLeftMotor.setPower(ARM_UP_POWER);
        armRightMotor.setPower(ARM_UP_POWER);
    }

    /**
     * This method with toggle the speed cap for forward and strafe
     */
    private double performSpeedCap(double goSpeed) {
        double capSpeed = 0;

        if (goSpeed < (-currentSpeedCap)) {
            capSpeed = -currentSpeedCap;
        } else if (goSpeed > currentSpeedCap) {
            capSpeed = currentSpeedCap;
        }
        return capSpeed;
    }


    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            //telemetry.addData("Claw Position F:R", "%7d:%7d", clampServoFront.getPosition(), clampServoRear.getPosition());
            telemetry.addData("Claw Position Front>", clampServoFront.getPosition());
            telemetry.addData("Arm Lift", String.format("%.01f cm", arm_lift_speed));
            telemetry.addData("Arm Left Motor Encoder Position", armLeftMotor.getCurrentPosition());
            telemetry.addData("Arm Right Motor Enc oder Position", armRightMotor.getCurrentPosition());
            telemetry.addData("Left Limit Switch is pressed?", !leftLimitSwitch.getState());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.update();
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
     * 
     * @param milliseconds
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * 
     */
    private void checkSpeedToggle(){
        speedToggleCurrent = gamepad1.left_bumper;

        if (speedToggleCurrent &&  (speedToggleCurrent != speedCapTogglePrev)) {
            if (currentSpeedCap == SPEED_CAP_MIN){
                currentSpeedCap = SPEED_CAP_MAX;
            }else {
                currentSpeedCap = SPEED_CAP_MIN;
            }
        }
        speedCapTogglePrev = speedToggleCurrent;

    }
    
    /**
     * This method will cut off the power to arm lift motor
     * if the limit switch is enabled and trying to move the motor down
     */
    private void limitArmDownMove(){
    
        if (leftLimitSwitch != null &&
            !leftLimitSwitch.getState() && 
               arm_lift_speed > 0){   
                    
           arm_lift_speed = 0;
        }

    }

}
