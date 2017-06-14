package org.firstinspires.ftc.teamcode11138;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static java.lang.Math.abs;
import static java.lang.Math.max;

/**
 * Created by 11138 Team on 11/5/2016.
 */
public class FTC11138Base1 extends LinearOpMode {
    static final double COUNTS_PER_MOTOR_REV = 1068;    // Neverest 40 motor: 7 counts/rev encoder X 40 reduction on shaft gear
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // Robot has another 1:2 gear. This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final float mmPerInch = 25.4f;
    static final float mmPerCm=10;
    static final float robotWidthInch = 18;
    static final float robotLengthInch = 18;
    static final float playFieldWidthInch = 141;  //11 feet 9 inch
    static final float playFieldLengthInch = 141;  //11 feet 9 inch

    //ToDo: The follow parameters need to be tested and adjusted, but applies to all running opmode.
    double blackColorValue=3.5; //This is the floor hsvValue reading.  Need to be tested on the play field
    float Kp_Drive = 0.002f;  //coefficient to adjust drive straight left and right motor power.
    float angleTolerance = 2;  //This is the tolerance of final position, within this tolerance, stop rotating.
    float stopDistanceMM = 120; //use this parameter if need to stop robot, avoid hitting something (wall, other robot, etc.)

    public ElapsedTime runtime = new ElapsedTime();
    FTC11138Robot1 robot = new FTC11138Robot1();
    ModernRoboticsI2cRangeSensor rangeSensor;
    BNO055IMU imu;
    ColorSensor beaconColorSensor;
    ColorSensor tapeColorSensor;

    Orientation imuAngles;
    Acceleration gravity;
    static double DRIVE_SPEED;
    static double TURN_SPEED;
    double refPositionAngle;
    double currentPositionAngle;

    VuforiaLocalizer myVuforiaLocalizer;
    VuforiaTrackables beaconTargets;
    VuforiaTrackable singleBeaconTarget;
    VuforiaTrackableDefaultListener listener;
    VuforiaLocalizer.Parameters params;

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void initIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void initMotors() throws InterruptedException {
       /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Send telemetry message to signify robot waiting;
        DRIVE_SPEED=0.3;
        TURN_SPEED=0.15;

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //// delete idle() from SDK 2.3 or later

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();
    }

    public DetectedColor detectedColor(ColorSensor colorSensorToUse) {
        DetectedColor sensorReadColor;

        double whiteToBlackValueRatio=1.2;  //ToDo: need to test on the field

        int redValue = colorSensorToUse.red();
        int blueValue = colorSensorToUse.blue();

        double hsvValue=getHSVValue(colorSensorToUse);

        if (hsvValue>whiteToBlackValueRatio*blackColorValue)
            sensorReadColor = DetectedColor.White;
        else if (hsvValue > 0.9*blackColorValue && hsvValue<1.1*blackColorValue)
            sensorReadColor=DetectedColor.Black;
        else if (redValue > blueValue)
                sensorReadColor = DetectedColor.Red;
            else sensorReadColor = DetectedColor.Blue;

        return sensorReadColor;
    }

    //use hsv Value to check for white
    public double getHSVValue(ColorSensor colorSensorToUse){
        int redValue = colorSensorToUse.red();
        int blueValue = colorSensorToUse.blue();
        int greenValue=colorSensorToUse.green();

        float hsvValues[]={0F,0F,0F};

        Color.RGBToHSV(redValue,greenValue,blueValue,hsvValues);
        return hsvValues[2];
    }

    public float getIMUAngle(){
        imuAngles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return(imuAngles.firstAngle);
    }

    public void turnRobot(Direction turnDirection, double turnDegree, double timeoutSec) {

        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        refPositionAngle = getIMUAngle();
        currentPositionAngle = refPositionAngle; //At the start, current position is the ref position.

        runtime.reset();

        while (abs(refPositionAngle - currentPositionAngle) < turnDegree && runtime.seconds()<timeoutSec) {
            //Turn 80% of the needed angle to turn unless the toleranceAngle is greater.

            double motorMoveStepInch = max(angleTolerance, (turnDegree - abs(currentPositionAngle - refPositionAngle)) * 0.8) / 360 * Math.PI * robotWidthInch;
            //rotate robot according to direction
            if (turnDirection == Direction.Left) {
                encoderDrive(TURN_SPEED, -motorMoveStepInch, motorMoveStepInch, timeoutSec);
            } else encoderDrive(TURN_SPEED, motorMoveStepInch, -motorMoveStepInch, timeoutSec);

            //get the angle reading at current position
            currentPositionAngle = getIMUAngle();
        }
    }

    public void turnRobot1Wheel(Direction turnDirection, double turnDegree, double timeoutSec) {

        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        refPositionAngle=getIMUAngle();
        currentPositionAngle=refPositionAngle; //At the start, current position is the ref position.

        runtime.reset();

        while (abs(refPositionAngle-currentPositionAngle)<turnDegree && runtime.seconds()<timeoutSec)
        {
            //Turn 80% of the needed angle to turn unless the toleranceAngle is greater.
            /*ToDo: Assume robot can move around center of it's front Axle.  Convert angleTolerance to linear move the wheel need to drive.
            If robot can only rotate around 1 wheel, need to use the robot width as the turn radius.
            */
            double motorMoveStepInch= max(angleTolerance,(turnDegree-abs(currentPositionAngle-refPositionAngle))*0.8)/360*Math.PI*robotWidthInch*2;
            //rotate robot according to direction
            if (turnDirection== Direction.Left) {
                encoderDrive(TURN_SPEED, 0, motorMoveStepInch, timeoutSec);
            }
            else encoderDrive(TURN_SPEED,motorMoveStepInch, 0, timeoutSec);

            //get the angle reading at current position
            currentPositionAngle=getIMUAngle();
        }
    }


    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftMotorTarget;
        int newRightMotorTarget;
        ElapsedTime driveTimer = new ElapsedTime(); //define a new timer just for the encoder drive, because functions calls encoderDrive has another runtimer

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftMotorTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightMotorTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftMotorTarget);
            robot.rightMotor.setTargetPosition(newRightMotorTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            driveTimer.reset();
            robot.leftMotor.setPower(abs(speed));
            robot.rightMotor.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (driveTimer.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftMotorTarget, newRightMotorTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    public void freeShoot(double speed) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.leftShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //start to run the motor
            robot.leftShoot.setPower(speed);
            robot.rightShoot.setPower(speed);

        } else {
            // Stop all motion;
            stopShoot();
        }
    }

    public void stopShoot() {
        robot.leftShoot.setPower(0);
        robot.rightShoot.setPower(0);
    }

    public void freeStraightDrive(double speed) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //start to run the motor
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

        } else {
            // Stop all motion;
            stopMotor();
        }
    }

    public void stopMotor() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
    public void adjustMotorPower(float expectedValue, float feedbackValue, float scalingFactor){

        double currentLeftMotorPower = robot.leftMotor.getPower();
        double currentRightMotorPower = robot.rightMotor.getPower();

        double powerToAdjust=(expectedValue-feedbackValue)*scalingFactor;

        //ToDo: need to confirm powerToAdjust polarity
        double newLeftMotorPower=currentLeftMotorPower-powerToAdjust;
        double newRightMotorPower=currentRightMotorPower+powerToAdjust;

        double maxPower=max(abs(newLeftMotorPower),abs(newRightMotorPower));

        if (maxPower>1) {
            newLeftMotorPower /= maxPower;
            newRightMotorPower /= maxPower;
        }

        robot.leftMotor.setPower(newLeftMotorPower);
        robot.rightMotor.setPower(newRightMotorPower);

        telemetry.addData("Expected value",expectedValue);
        telemetry.addData("current value",feedbackValue);
        telemetry.addData("left motor power", newLeftMotorPower);
        telemetry.addData("right motor power", newRightMotorPower);
        telemetry.update();
    }

    public void driveAlongTape(DetectedColor tapeColor, Direction tapeEdge){
        double currentLeftMotorPower = robot.leftMotor.getPower();
        double currentRightMotorPower = robot.rightMotor.getPower();
        double newLeftMotorPower;
        double newRightMotorPower;
        double powerToAdjust=(tapeEdge==Direction.Left)? 0.02: -0.02; //ToDo: need to confirm powerToAdjust polarity
        if (detectedColor(tapeColorSensor)==tapeColor) {

            newLeftMotorPower = currentLeftMotorPower - powerToAdjust;
            newRightMotorPower = currentRightMotorPower + powerToAdjust;
        }
        else {
            newLeftMotorPower = currentLeftMotorPower + powerToAdjust;
            newRightMotorPower = currentRightMotorPower - powerToAdjust;
        }

        double maxPower=max(abs(newLeftMotorPower),abs(newRightMotorPower));

        if (maxPower>1) {
            newLeftMotorPower /= maxPower;
            newRightMotorPower /= maxPower;
            }

        robot.leftMotor.setPower(newLeftMotorPower);
        robot.rightMotor.setPower(newRightMotorPower);
    }

    public void setupVuforia() {
        // Setup parameters to create localizer
        params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = "Aa96UGf/////AAAAGcKOfzCYjkCzoSoNIzMvr8JRzY5qFtNcAcFmzlil1nQBvum1Ny6PlTkJ0HG6wazv53CSNd9oduRnGR457STwMjGDzgTdsg3dPW9aZZfaLpeU3JvNxJG0j3dtksf33TS0/x1wbQZJ91rJ+KEdei4InW13KWZGC3XwJHApkha2fpj6JbckRVJxzWMykbZ3IjlcZ0XlmXZR2vn0lFrdzLHUQctW0pHvJ2h4CRl11qgfnhVJF20gDLNhU7zSUfo6Sr9mByItdfSeDilWdy3oDEWr26wiUJUaRdMv+CYbTL81LH+Pxxz367xmrj+LwO1OLBAAG9yANvsQaK9YnfjO6gWDNtdSJ31SfNOrHT0KfOZ4ndzE"; // Insert your own key here
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        myVuforiaLocalizer = ClassFactory.createVuforiaLocalizer(params);
    }

    public void setupTarget(int targetNumber) {
        //Load targets//
        String targetNames[]={"wheels","tools","legos","gears"};
        beaconTargets = myVuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);  //Just track 1 image on the left//

        // Setup the target to be tracked
        singleBeaconTarget = beaconTargets.get(targetNumber); // 3 corresponds to gears
        singleBeaconTarget.setName(targetNames[targetNumber]);

        //ToDo: Put target location in array, based on targetNumber, use the right parameters
        final float beaconTargetLocationXmm=-playFieldWidthInch*mmPerInch/2;
        final float beaconTargetLocationYmm=-playFieldWidthInch*mmPerInch/12;
        final float beaconTargetLocationZmm=0;
        final float beaconTargetRotationUdeg=90;
        final float beaconTargetRotationVdeg=0;
        final float beaconTargetRotationWdeg=90;

        OpenGLMatrix beaconTargetLocation = createMatrix(   beaconTargetLocationXmm,
                beaconTargetLocationYmm,
                beaconTargetLocationZmm,
                beaconTargetRotationUdeg,
                beaconTargetRotationVdeg,
                beaconTargetRotationWdeg);

        singleBeaconTarget.setLocation(beaconTargetLocation);
    }

    public void setupPhone() {
        /* Set phone location on robot
        Robot origin at the middle of robot
        X: to the left
        Y: to the front
        Z: vertical up
        Looking into origin along any axes, CCW is positive angle.

        Phone original position is laying flat on robot, screen facing up, short side parallel to robot front edge, long side parallel to robot right edge.
        Phone location: Middle, Front of robot, screen facing inside, landscape.
        Movement from original position is:
            move +Y half robot length
            flip 90 degree around X
        Phone location using mm.
         */
        OpenGLMatrix phoneLocation = createMatrix(0, robotLengthInch * mmPerInch / 2, 0, 90, -90, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) singleBeaconTarget.getListener();
        listener.setPhoneInformation(phoneLocation, params.cameraDirection);
    }

    public double getAngleToTurn() {
        //Based on Vuforia reading, return the angles need to turn to align to center of beacon target.
        double beaconTargetAngle;
        //feedback if phone can see the target
        telemetry.addData(singleBeaconTarget.getName(), ((VuforiaTrackableDefaultListener) singleBeaconTarget.getListener()).isVisible() ? "Visible" : "Not Visible");    //
        OpenGLMatrix robotLocation = ((VuforiaTrackableDefaultListener) singleBeaconTarget.getListener()).getUpdatedRobotLocation();
        if (robotLocation != null) {
            telemetry.addData("Pos", format(robotLocation)); //This should be a string containing xyz and uvw data.  Need to parse to get the data.

            /*Another method is to use vector, which should contain xyz data
            use getPose method,which returns data in phone's coordinate.  If phone in landscape mode:
            lef/right: Y value change.  0 is aligned.
            up/down: X value change.
            forward/backward: z valude change.  Always negative because +Z is pointing away from screen (to user direction)
             */
            VectorF beaconTargetVector = ((VuforiaTrackableDefaultListener) singleBeaconTarget.getListener()).getPose().getTranslation();
            //Phone in landscape mode, use Y and Z to calculate target to phone angle
            //If Phone in portrait mode, use X and Z to calculate angles to turn

            beaconTargetAngle = Math.toDegrees(Math.atan2(beaconTargetVector.get(1), Math.abs(beaconTargetVector.get(2)))); //use abs to make Z positive.  Then the calculated angle will be centered around 0 degree.

            telemetry.addLine(singleBeaconTarget.getName() + "Degrees to Turn:" + beaconTargetAngle); //show the angle calculated by XYZ coordinate
        } else {
            telemetry.addData("Pos", "Unknown");
            telemetry.addLine("Keep current direction"); //ToDo: This may not be right as if don't turn, may never find target.
            beaconTargetAngle = 45;
        }
        telemetry.update();
        return(beaconTargetAngle);
    }

    public OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w) {
        return OpenGLMatrix.translation(x, y, z).multiplied(Orientation.getRotationMatrix(
                AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    enum Direction {Left, Right}

    enum DetectedColor {Blue, Red, Black, White}
}
