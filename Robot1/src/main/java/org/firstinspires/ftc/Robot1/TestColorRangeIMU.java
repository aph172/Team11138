/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.Robot1;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * the Adafruit RGB Sensor.  It assumes that the I2C
 * cable for the sensor is connected to an I2C port on the
 * Core Device Interface Module.
 *
 * It also assuems that the LED pin of the sensor is connected
 * to the digital signal pin of a digital port on the
 * Core Device Interface Module.
 *
 * You can use the digital port to turn the sensor's onboard
 * LED on or off.
 *
 * The op mode assumes that the Core Device Interface Module
 * is configured with a name of "dim" and that the Adafruit color sensor
 * is configured as an I2C device with a name of "color".
 *
 * It also assumes that the LED pin of the RGB sensor
 * is connected to the signal pin of digital port #5 (zero indexed)
 * of the Core Device Interface Module.
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Test Color Range IMU", group = "Test")

// @Disabled                            // Comment this out to add to the opmode list
public class TestColorRangeIMU extends FTC11138Base2 {

  @Override
  public void runOpMode() throws InterruptedException {

    robot.init(hardwareMap);
    getInitValues();

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues_MR[] = {0F,0F,0F};
    float hsvValues_Adafruit[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values_MR[] = hsvValues_MR;
    final float values_Adafruit[] = hsvValues_Adafruit;
    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bAdaLedOn = true;
    boolean bMRLedOn = false;
    I2cAddr I2c_MR, I2c_Adafruit;

    // set the digital channel to output mode.
    // remember, the Adafruit sensor is actually two devices.
    // It's an I2C sensor and it's also an LED that can be turned on or off.
    robot.cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

    I2c_MR = robot.beaconColorSensor.getI2cAddress();
    I2c_MR = robot.colorSensor2.getI2cAddress();

    //turn off the LED on MR sensor
    robot.beaconColorSensor.enableLed(bMRLedOn);
    //turn Adafruit LED
    robot.cdim.setDigitalChannelState(LED_CHANNEL, bAdaLedOn);

    // wait for the start button to be pressed.
    waitForStart();

    // loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive())  {


      // check the status of the x button on gamepad.
      bCurrState = gamepad1.x;

      // check for button-press state transitions.
      if ((bCurrState) && (bCurrState != bPrevState))  {

        // button is transitioning to a pressed state. Toggle the LED.
        bMRLedOn = !bMRLedOn;
        robot.beaconColorSensor.enableLed(bMRLedOn);
        //cdim.setDigitalChannelState(LED_CHANNEL, bAdaLedOn);
      }

      // update previous state variable.
      bPrevState = bCurrState;

      // convert the RGB values to HSV values.
      Color.RGBToHSV( robot.colorSensor2.red(),
                      robot.colorSensor2.green(),
                      robot.colorSensor2.blue(), hsvValues_MR);
      Color.RGBToHSV( robot.beaconColorSensor.red(),
                      robot.beaconColorSensor.green(),
                      robot.beaconColorSensor.blue(), hsvValues_MR);

      // send the info back to driver station using telemetry function.
      telemetry.addData("tape sensor detected color", detectedGroundColor(robot.colorSensor2));
      telemetry.addData("beacon sensor detected color", detectedBeaconColor(robot.beaconColorSensor));

      telemetry.addData("LED", bMRLedOn ? "On" : "Off");
      telemetry.addData("MR: Clear", robot.beaconColorSensor.alpha());
      telemetry.addData("MR: Red  ", robot.beaconColorSensor.red());
      telemetry.addData("MR: Green", robot.beaconColorSensor.green());
      telemetry.addData("MR: Blue ", robot.beaconColorSensor.blue());
      telemetry.addData("MR: Hue", hsvValues_MR[0]);
      telemetry.addData("MR Sat", hsvValues_MR[1]);
      telemetry.addData("MR Value",hsvValues_MR[2]);
      telemetry.addData("MR I2c Address: ", I2c_MR.toString());

      telemetry.addData("LED", bMRLedOn ? "On" : "Off");
      telemetry.addData("MR: Clear", robot.colorSensor2.alpha());
      telemetry.addData("MR: Red  ", robot.colorSensor2.red());
      telemetry.addData("MR: Green", robot.colorSensor2.green());
      telemetry.addData("MR: Blue ", robot.colorSensor2.blue());
      telemetry.addData("MR: Hue", hsvValues_MR[0]);
      telemetry.addData("MR Sat", hsvValues_MR[1]);
      telemetry.addData("MR Value",hsvValues_MR[2]);
      telemetry.addData("MR I2c Address: ", I2c_MR.toString());

      telemetry.addData("raw ultrasonic", robot.rangeSensor.rawUltrasonic());
      telemetry.addData("raw optical", robot.rangeSensor.rawOptical());
      telemetry.addData("cm optical", "%.2f cm", robot.rangeSensor.cmOptical());
      telemetry.addData("cm", "%.2f mm", robot.rangeSensor.getDistance(DistanceUnit.MM));

      telemetry.addData("reference angle", startPositionAngle);
      telemetry.addData("IMU reading", getIMUAngle());
      telemetry.addData("Turn left 45, actual angle need to turn", calcAngleToTurn(startPositionAngle,45, Direction.Left));
      telemetry.addData("Turn right 45, actual angle need to turn", calcAngleToTurn(startPositionAngle,45, Direction.Right));

      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values_MR));
        }
      });

      telemetry.update();
      // delete idle() from SDK 2.3 or later // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }
  }
}
