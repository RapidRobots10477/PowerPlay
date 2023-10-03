package org.firstinspires.ftc.teamcode.centerstage.drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.teamcode.PIDController;
    //This is with multithreading
    @TeleOp(name="TeleOp DriveCenterStageGame")
    public class TeleOpCenterstage extends OpMode {

        // create motor objects and initialize to null
        private DcMotor leftFrontMotor = null;
        private DcMotor rightFrontMotor = null;
        private DcMotor leftBackMotor = null;
        private DcMotor rightBackMotor = null;
        //private DcMotor armLiftMotor = null;
        private BNO055IMU imu = null;      // Control Hub IMU

        private PIDController pidRotate, pidDrive;

        // create Claw Servo object and initialize to null
        //Servo clawServo = null;
        private ColorSensor colorSensorBottom    = null;
        private DistanceSensor distanceSensorFront = null;
        boolean speed_lmt_on;
        boolean servoCheck;

        double driveSpeed, currentSpeedCap, forward, strafe;
        double rotate;

        static final double SPEED_CAP_MIN    = 0.5;
        static final double SPEED_CAP_MAX    = 0.75;
        static final int START_POS           = 0;

        static final double ROTATE_SPEED = 0.75;        // Max rotate speed to limit turn rate
        // Max rotate speed to limit turn ate
        static final double TURN_SPEED = 0.4;           // Max Turn speed to limit turn rate
        static final double HEADING_THRESHOLD = 1.0;    // the target before moving to next step.
        static final double P_TURN_GAIN = 0.02;         // Larger is more responsive, but also less stable
        static final double P_DRIVE_GAIN = 0.03;        // Larger is more responsive, but also less stable

        private double targetHeading = 0;
        private double robotHeading  = 0;
        private double headingOffset = 0;
        private double headingError  = 0;
        private double turnSpeed     = 0;
        private double leftSpeed     = 0;
        private double rightSpeed    = 0;

        boolean rotate_check, rotate_check_press;

        boolean speedToggleCurrent, speedCapTogglePrev  = false;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        Thread thread = null;

        /**
         *
         * Please dont delete this. This is the last resort for multi-threading
         * if the existing mode is not working
         */
        //Task moveArmTask = new Task();
        //Thread taskT = new Thread(moveArmTask);


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

            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            pidRotate = new PIDController(.003, .00003, 0);

            currentSpeedCap = SPEED_CAP_MIN;
            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
            rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        }

        @Override
        public void loop() {
            // Read motor encoder values for the Arm
            //arm_lift_motor_pos = armLiftMotor.getCurrentPosition();
            /* Read control button status for base vehicle movement */
            forward = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

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
         * Method to spin on central axis to point in a new direction.
         * Move will stop if either of these conditions occur:
         * 1) Move gets to the heading (angle)
         *
         * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
         * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
         *                     0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
         *                     If a relative angle is required, add/subtract from current heading.
         */
        public void turnToHeading(double maxTurnSpeed, double heading) {

            // Run getSteeringCorrection() once to pre-calculate the current error
            getSteeringCorrection(heading, P_DRIVE_GAIN);

            // keep looping while we are still active, and not on heading.
            while (Math.abs(headingError) > HEADING_THRESHOLD) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

                // Clip the speed to the maximum permitted value.
                turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

                // Pivot in place by applying the turning correction
                moveRobot(0, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }
        }

        /**
         * This method uses a Proportional Controller to determine how much steering correction is required.
         *
         * @param desiredHeading   The desired absolute heading (relative to last heading reset)
         * @param proportionalGain Gain factor applied to heading error to obtain turning power.
         * @return Turning power needed to get to required heading.
         */
        public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
            targetHeading = desiredHeading;  // Save for telemetry

            // Get the robot heading by applying an offset to the IMU heading
            robotHeading = getRawHeading() - headingOffset;

            // Determine the heading current error
            headingError = targetHeading - robotHeading;

            // Normalize the error to be within +/- 180 degrees
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
            return Range.clip(headingError * proportionalGain, -1, 1);
        }

        /**
         * read the raw (un-offset Gyro heading) directly from the IMU
         */
        public double getRawHeading() {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return angles.firstAngle;
        }

        /**
         * Reset the "offset" heading back to zero
         */
        public void resetHeading() {
            // Save a new heading offset equal to the current raw heading.
            //imu.initialize(parameters);
            headingOffset = getRawHeading();
            robotHeading = 0;
        }

        /**
         * Display the various control parameters while driving
         *
         * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
         */
        private void sendTelemetry(boolean straight) {

            if (straight) {
                //telemetry.addData("Claw Position F:R", "%7d:%7d", clampServoFront.getPosition(), clampServoRear.getPosition());
                telemetry.addData("Actual Pos FL:FR", "%7d:%7d", leftFrontMotor.getCurrentPosition(), rightFrontMotor.getCurrentPosition());
                telemetry.addData("Actual Pos RL:RR", "%7d:%7d", leftBackMotor.getCurrentPosition(), rightBackMotor.getCurrentPosition());
            } else {
                telemetry.addData("Motion", "Turning");
            }

            telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
            telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
            telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
            telemetry.update();
        }

        /**
         * Open Claw
         */

        private void rotate(double power, int degrees)
        {
            // restart imu angle tracking.
            resetHeading();

            // if degrees > 359 we cap at 359 with same sign as original degrees.
            if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

            pidRotate.reset();
            pidRotate.setSetpoint(degrees);
            pidRotate.setInputRange(0, degrees);
            pidRotate.setOutputRange(0, power);
            pidRotate.setTolerance(1);
            pidRotate.enable();
            ElapsedTime holdTimer = new ElapsedTime();
            holdTimer.reset();
            if (degrees < 0)
            {
                // On right turn we have to get off zero first.
                //while ((holdTimer.time() < 1000) && getRawHeading() == 0)
                // {
                //moveRobot(rotate, degrees);
                // }
                holdTimer.reset();
                do
                {
                    power = pidRotate.performPID(getRawHeading()); // power will be - on right turn.
                    leftFrontMotor.setPower(power);
                    rightFrontMotor.setPower(-power);
                    leftBackMotor.setPower(power);
                    rightBackMotor.setPower(-power);
                } while ((holdTimer.time() < 3000) && !pidRotate.onTarget());
            }
            else    // left turn.
                holdTimer.reset();
            do
            {
                power = pidRotate.performPID(getRawHeading()); // power will be + on left turn.
                leftFrontMotor.setPower(power);
                rightFrontMotor.setPower(-power);
                leftBackMotor.setPower(power);
                rightBackMotor.setPower(-power);
            } while ((holdTimer.time() < 2000) && !pidRotate.onTarget());

            // turn the motors off.
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);

            robotHeading = getRawHeading();

            // wait for rotation to stop.
            sleep(500);
            // reset angle tracking on new heading.
            resetHeading();
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
 class Task implements Runnable{
 int position = 0;

 void setPosition(int holdPosition){
 position = holdPosition;
 }
 @Override
 public void run() {
 while (Math.abs(armLeftMotor.getCurrentPosition()) < Math.abs(position) ||
 Math.abs(armRightMotor.getCurrentPosition()) < Math.abs(position)) {
 armLeftMotor.setPower(ARM_UP_POWER);
 armRightMotor.setPower(ARM_UP_POWER);
 telemetry.addData("ArmLeftPosition>", armLeftMotor.getCurrentPosition());
 telemetry.addData("ArmRightPosition>", armRightMotor.getCurrentPosition());
 telemetry.update();
 }
 armLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 armRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 }

 }

 **/

    }

