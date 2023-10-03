package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TestArmAndClaw")
//@Disabled
public class TestArmAndClaw extends OpMode {
//static final doubles contain the final value;no method needed to back it up
    private DcMotor leftFrontMotor  = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor   = null;
    private DcMotor rightBackMotor  = null;
    private DcMotor armLeftMotor    = null;
    private DcMotor armRightMotor   = null;
    private DcMotor armLiftMotor    = null;

    private Servo clampServoFront   = null;
    private Servo clampServoRear    = null;

    boolean capPressed;
    double capSpeed;

    double topLeftDiagonal;
    double topRightDiagonal;
    double forward;
    double strafe;
    double rotate;

    double arm_lift_speed;
    double arm_height;

    

    static final double FRONT_CLAW_OPEN  = 0.80;
    static final double FRONT_CLAW_CLOSE = 0.5;
    static final double REAR_CLAW_OPEN   = 0.80;
    static final double REAR_CLAW_CLOSE  = 0.45;


    int arm_left_motor_pos;
    int arm_right_motor_pos;
    int arm_lift_motor_pos;
    int leftFrontMotor_pos;
    int rightFrontMotor_pos;
    int leftBackMotor_pos;
    int rightBackMotor_pos;
    //int arm_extend_motor_pos;

    boolean speedCapButton;

    @Override
    public void init() {
        /* Map all hardware ports */
        /* Macanum Wheels motors */
        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front");
        leftBackMotor = hardwareMap.dcMotor.get("left_rear");
        rightBackMotor = hardwareMap.dcMotor.get("right_rear");

        clampServoFront= hardwareMap.get(Servo.class, "front_clamp");
        clampServoRear = hardwareMap.get(Servo.class, "rear_clamp");
        closeClaw();
        
        //armLiftMotor = hardwareMap.dcMotor.get("left_arm");
        armLeftMotor = hardwareMap.dcMotor.get("left_arm");
        armRightMotor = hardwareMap.dcMotor.get("right_arm");        //armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        capPressed = false;
        capSpeed = 0.50;
    }

    @Override
    public void loop() {

        if(speedCapButton == false) {
            capPressed = false;
        }

        // Read motor encoder values for the Arm
        //arm_lift_motor_pos = armLiftMotor.getCurrentPosition();
        arm_left_motor_pos = armLeftMotor.getCurrentPosition();
        arm_right_motor_pos = armRightMotor.getCurrentPosition();
        leftFrontMotor_pos = leftFrontMotor.getCurrentPosition();
        rightFrontMotor_pos = rightFrontMotor.getCurrentPosition();
        leftBackMotor_pos = leftBackMotor.getCurrentPosition();
        rightBackMotor_pos = rightBackMotor.getCurrentPosition();

        forward = gamepad1.left_stick_y; // 0
        strafe = gamepad1.left_stick_x; // 1
        rotate = gamepad1.right_stick_x;

        speedCapButton = gamepad1.left_bumper;

        // Read arm lift

        /* Read control button status for base vehicle movement */

        arm_lift_speed = gamepad2.left_stick_y;


        topLeftDiagonal = forward - strafe; // + =
        topRightDiagonal = forward + strafe;//- =


        if (rotate > 0) {
            if (rotate > 0.5) {
                rotate = 0.5;
            }
            leftFrontMotor.setPower(-rotate);
            rightBackMotor.setPower(-rotate);//negs

            rightFrontMotor.setPower(-rotate);//negs
            leftBackMotor.setPower(-rotate);
        }
        if (rotate < 0) {
            if (rotate < -0.5) {
                rotate = -0.75;
            }
            leftFrontMotor.setPower(-rotate);
            rightBackMotor.setPower(-rotate);//negs

            rightFrontMotor.setPower(-rotate);//negs
            leftBackMotor.setPower(-rotate);
        }

        if(speedCapButton == true && capSpeed == 0.25 && capPressed == false) {
            capPressed = true;
            capSpeed = 0.50;
            telemetry.addData("Speed : ", capSpeed);


        }

        if(speedCapButton == true && capSpeed == 0.50 && capPressed == false) {
            capPressed = true;
            capSpeed = 0.25;
            telemetry.addData("Speed : ", capSpeed);

        }

        telemetry.addData("Speed : ", capSpeed);

        if (topLeftDiagonal > capSpeed)
        {
            topLeftDiagonal = capSpeed;
            telemetry.addData("TopLeftDiagonal",topLeftDiagonal);
        }
        if (topLeftDiagonal < -capSpeed)
        {
            topLeftDiagonal = -capSpeed;
        }
        if (topRightDiagonal > capSpeed)
        {
            topRightDiagonal = capSpeed;
        }
        if (topRightDiagonal < -capSpeed)
        {
            topRightDiagonal = -capSpeed;
        }
        telemetry.addData("TopLeftDiagonal",topLeftDiagonal);
        telemetry.addData("TopRightDiagonal",topRightDiagonal);

        if (gamepad2.a == true) {
            arm_lift_speed = -0.32;
            telemetry.addData("Held speed", arm_lift_speed);
        }

        arm_lift_speed = gamepad2.left_stick_y;

        if (arm_lift_speed < -0.75) {
            arm_lift_speed = -0.75;
        }
        if (arm_lift_speed > 0.75) {
            arm_lift_speed = 0.75;
        }

        armLeftMotor.setPower(arm_lift_speed);
        armRightMotor.setPower(-arm_lift_speed);
        
        //armRightMotor.setPower(arm_lift_speed);

        leftFrontMotor.setPower(topLeftDiagonal);
        rightBackMotor.setPower(-topLeftDiagonal);//-

        rightFrontMotor.setPower(-topRightDiagonal);//-
        leftBackMotor.setPower(topRightDiagonal);
        
        if (gamepad2.left_bumper) {
            closeClaw();
        }

        if (gamepad2.right_bumper) {
            openClaw();
        }

        /* Update global telemetry status */
        telemetry.addData("Arm Movement Speed", arm_lift_speed);
        telemetry.addData("Claw Front Position>", clampServoFront.getPosition());
        telemetry.addData("Claw Rear Position>", clampServoRear.getPosition());
        telemetry.addData("Arm Lift", String.format("%.01f cm", arm_height));
        telemetry.addData("Arm Left Motor Encoder Position", arm_left_motor_pos);
        telemetry.addData("Arm Right Motor Encoder Position", arm_right_motor_pos);
        //telemetry.addData("Arm Extend Motor Encoder Position", arm_extend_motor_pos);

        telemetry.addData("Left Front Encoder Position", leftFrontMotor_pos);
        telemetry.addData("Right Front Encoder Position", rightFrontMotor_pos);
        telemetry.addData("Left Back Encoder Position", leftBackMotor_pos);
        telemetry.addData("Right Back Encoder Position", rightBackMotor_pos);

        telemetry.update();
    }
    
    private void openClaw() {
        clampServoFront.setPosition(FRONT_CLAW_OPEN);    
        clampServoRear.setPosition(REAR_CLAW_OPEN);      
    }
    
    private void closeClaw(){
        clampServoFront.setPosition(FRONT_CLAW_CLOSE);    
        clampServoRear.setPosition(REAR_CLAW_CLOSE);     
    }

}


