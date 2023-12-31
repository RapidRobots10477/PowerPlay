package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/*
 *
 */
@TeleOp(name = "REVTouchSensor Calibrate", group = "Sensor")
@Disabled
public class TeleOpCalibrateTouchSensor extends LinearOpMode {
    /**
     * The REV Robotics Touch Sensor
     * is treated as a digital channel.  It is HIGH if the button is unpressed.
     * It pulls LOW if the button is pressed.
     *
     * Also, when you connect a REV Robotics Touch Sensor to the digital I/O port on the
     * Expansion Hub using a 4-wire JST cable, the second pin gets connected to the Touch Sensor.
     * The lower (first) pin stays unconnected.*
     */

    DigitalChannel digitalTouchLeft, digitalTouchRight;  // Hardware Device Object

    @Override
    public void runOpMode() {

        // get a reference to our digitalTouch object.
        digitalTouchLeft  = hardwareMap.get(DigitalChannel.class, "left_limit");
        digitalTouchRight = hardwareMap.get(DigitalChannel.class, "right_limit");

        // set the digital channel to input.
        digitalTouchLeft.setMode(DigitalChannel.Mode.INPUT);
        digitalTouchRight.setMode(DigitalChannel.Mode.INPUT);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.
            if (digitalTouchLeft.getState() == true) {
                telemetry.addData("Digital Touch Left", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch Left", "Is Pressed");
            }
            if (digitalTouchRight.getState() == true) {
                telemetry.addData("Digital Touch Right", "Is Not Pressed");
            } else {
                telemetry.addData("Digital Touch Right", "Is Pressed");
            }

            telemetry.update();
        }
    }
}
