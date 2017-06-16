/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.Robot2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="FTC11138PushBot", group="11138")

public class FTC11138PushBot extends LinearOpMode {
    /* Declare OpMode members. */
    FTC11138Robot2 myRobot          = new FTC11138Robot2();   // Use a Pushbot's hardware
                                                            // could also use HardwarePushbotMatrix class.
    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double turnRight;
        double turnLeft;
        boolean shoot;
        boolean dispenser;
        boolean approachTouch;
        boolean collector;
        boolean collector2;
        double collectorPower = 0;  //collector motor power
        double    clawOffset      = 0.0;

        myRobot.initTeleop(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, 0.02, 0.98);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        myRobot.rightShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(20);
        while (opModeIsActive()) {
                        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            //left  = gamepad1.left_stick_y/1.5;  //ToDo: reduce power because robot's gear ratio
            //right = gamepad1.right_stick_y/1.5;
            left = gamepad1.right_stick_y/1.5;
            right = gamepad1.left_stick_y/1.5;
            //turnLeft = gamepad1.left_stick_x/1.5;
            //turnRight = -gamepad1.right_stick_x/1.5;
            //right  = -gamepad1.left_stick_y/1.7;  //ToDo: reduce power because robot's gear ratio
            //left = -gamepad1.right_stick_y/1.7;
            //shoot = gamepad2.b;
            approachTouch = gamepad2.b;
            shoot = gamepad2.y;
            if(approachTouch) {
                myRobot.rightShoot.setPower(0.24);
                while(!myRobot.shootTouchSensor.isPressed()) {
                    sleep(1);
                }
                myRobot.rightShoot.setPower(0);
            }
            else if(shoot) {
                myRobot.rightShoot.setPower(0.3);
                sleep(400);
                myRobot.rightShoot.setPower(0);
            }
            else {
                myRobot.rightShoot.setPower(0);
            }


            collector = gamepad2.right_bumper;
            collector2 = gamepad2.left_bumper;
            if (collector)
                collectorPower = -1.0;
            else if (collector2)
                collectorPower = 1.0;
            else
                collectorPower = 0.0;

            clawOffset = gamepad2.a? 0.9: 0.15;

            //myRobot.leftMotor.setPower(right);
            myRobot.leftMotor.setPower(left);
            //myRobot.rightMotor.setPower(left);
            myRobot.rightMotor.setPower(right);
            //myRobot.rightShoot.setPower(shoot?-0.4:0);

            myRobot.collectorMotor.setPower(collectorPower);
            myRobot.dispensingServo.setPosition(clawOffset);
            //myRobot.buttonPusherLeft.setPosition(clawOffset2);

            telemetry.addData("Servo position:", clawOffset);
            //telemetry.addData("Button Pusher Position", clawOffset2);
            telemetry.addData("Left Y value", gamepad1.left_stick_y);
            telemetry.addData("Left X value", gamepad1.left_stick_x);
            telemetry.addData("Right Y value", gamepad1.right_stick_y);
            telemetry.addData("Right X value", gamepad1.right_stick_x);
            telemetry.addData("Shoot b value", gamepad2.b);
            telemetry.addData("Left Bumper", gamepad2.left_bumper);
            telemetry.addData("Right Bumper", gamepad2.right_bumper);
            telemetry.addData("A value", gamepad2.a);
            telemetry.addData("Y Value", gamepad2.y);


            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            //sleep(40);
        }
    }
}
