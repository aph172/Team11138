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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

// @Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
@TeleOp(name="Test Auto Drive By Encoder and Turn", group="Test")
// @Disabled
public class TestDriveByEncoderAndTurn extends FTC11138Base2 {

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        getInitValues();
        double angleToTurn;

        DRIVE_SPEED = 0.25;
        TURN_SPEED = 0.24;

        telemetry.addData("drive speed", DRIVE_SPEED);
        telemetry.addData("turn speed", TURN_SPEED);
        telemetry.addData("start angle", startPositionAngle);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDrive(DRIVE_SPEED, 24, 24, 8.0);  // S1: Forward 24 Inches with 8 Sec timeout

        telemetry.addData("Next: Turn 2 Wheels", "Right");
        telemetry.update();
        sleep(1000);

        //turn right 45 degree from start angle direction
        angleToTurn = calcAngleToTurn(startPositionAngle, 45, Direction.Right);

        turnRobot(TURN_SPEED, Direction.Right, angleToTurn, 10);

        telemetry.addData("start position angle", startPositionAngle);
        telemetry.addData("Planned angle to turn from start position", 45);
        telemetry.addData("actual angle to turn", angleToTurn);
        telemetry.addData("current position angle", currentPositionAngle);
        telemetry.addData("angle have rotated", Math.abs(currentPositionAngle - startPositionAngle));
        telemetry.addData("Next: Turn one Wheel", "Right");
        telemetry.update();
        sleep(2000);

        angleToTurn = calcAngleToTurn(startPositionAngle, 90, Direction.Right);
        turnRobot1Wheel(TURN_SPEED, Direction.Right, angleToTurn, 10);
        telemetry.addData("start position angle", startPositionAngle);
        telemetry.addData("Planned angle to turn from last position", 45);
        telemetry.addData("actual angle to turn", angleToTurn);
        telemetry.addData("current position angle", currentPositionAngle);
        telemetry.addData("angle have rotated since start", Math.abs(currentPositionAngle - startPositionAngle));
        telemetry.update();
        sleep(2000);

        angleToTurn = calcAngleToTurn(startPositionAngle, 45, Direction.Right);
        turnRobot1Wheel(TURN_SPEED, Direction.Right, angleToTurn, 10);
        telemetry.addData("start position angle", startPositionAngle);
        telemetry.addData("Planned angle to turn from last position", 45);
        telemetry.addData("actual angle to turn", angleToTurn);
        telemetry.addData("current position angle", currentPositionAngle);
        telemetry.addData("angle have rotated since start", Math.abs(currentPositionAngle - startPositionAngle));
        telemetry.update();
        sleep(2000);
    }
}
