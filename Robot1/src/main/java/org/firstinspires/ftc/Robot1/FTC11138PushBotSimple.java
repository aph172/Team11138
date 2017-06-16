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
package org.firstinspires.ftc.Robot1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="FTC11138 PushBot2", group="11138")

public class FTC11138PushBotSimple extends LinearOpMode {

    /* Declare OpMode members. */
    FTC11138Robot1 myRobot          = new FTC11138Robot1();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    @Override
    public void runOpMode() throws InterruptedException {
        double left;
        double right;
        double turnRight;
        double turnLeft;
        //double max;
        boolean shoot;
        boolean collector;
        boolean collector2;
        double    clawOffset      = 0.0;
        //double  clawOffset2    = 0.0;
        //double    CLAW_SPEED      = 0.1 ;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        myRobot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, 0.02, 0.98);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            left  = Range.clip(-gamepad1.left_stick_y/1.5,-1.0, 1.0);  //ToDo: reduce power because robot's gear ratio
            right = Range.clip(-gamepad1.right_stick_y/1.5,-1.0, 1.0);

            //myRobot.leftMotor.setPower(right);
            myRobot.leftMotor.setPower(left);
            //myRobot.rightMotor.setPower(left);
            myRobot.rightMotor.setPower(right);

            //telemetry.addData("Button Pusher Position", clawOffset2);
            telemetry.addData("Left Y value", gamepad1.left_stick_y);
            telemetry.addData("Right Y value", gamepad1.right_stick_y);
            telemetry.addData("Shoot b value", gamepad2.b);
            telemetry.addData("Left Bumper", gamepad2.left_bumper);
            telemetry.addData("Right Bumper", gamepad2.right_bumper);

            telemetry.update();

        }
    }
}
