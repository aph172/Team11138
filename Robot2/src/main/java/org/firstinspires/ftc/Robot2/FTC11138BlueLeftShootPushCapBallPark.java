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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "FTC11138BlueLeftShootPushCapBallPark", group = "11138")

public class FTC11138BlueLeftShootPushCapBallPark extends FTC11138Base2 {

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        getInitValues();
        robot.dispensingServo.setPosition(0.15);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        sleep(10000);
        encoderDrive(0.34, -11, -11, 10.0);
        shootTwoParticles();
        encoderDrive(0.32, -15, -15, 10.0);
        sleep(1000);
        Direction turnDirection = Direction.Left;
        turnRobotRightWheelMorePower(0.33, turnDirection, 100, 10.0);
        encoderDrive(0.34, 70, 70, 10.0);
        //turnRobot(0.33, turnDirection, 90, 10.0);
        //turnRobot(0.33, turnDirection, 180, 10.0);
        //encoderDrive(0.34, 45, 45, 10.0);
        //noEncoderDetectWhiteLine();


    }
}
