package org.firstinspires.ftc.Robot2;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static java.lang.Math.*;

/**
 * This is NOT an opmode.
 *
 * This class defines the components robot our team is going to build.
 *
 */
public class FTC11138Robot2
{
    /* Public OpMode members. */
    private HardwareMap hwMap  = null;
    public DcMotor  leftMotor  = null;
    public DcMotor  rightMotor = null;
    public DcMotor  rightShoot = null;
    public DcMotor  collectorMotor = null;
    public Servo    dispensingServo = null;

    public ModernRoboticsI2cRangeSensor rangeSensor = null;
    public BNO055IMU imu = null;
    public ColorSensor beaconColorSensor = null;
    public ColorSensor colorSensor2 = null;
    public DeviceInterfaceModule cdim = null;
    public TouchSensor shootTouchSensor = null;

    /* Constructor */
    public FTC11138Robot2(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
    // Save reference to Hardware map
    hwMap = ahwMap;

    beaconColorSensor = hwMap.colorSensor.get("mr color sensor");//Modern Robotics Color sensor
    rangeSensor=hwMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
    colorSensor2 = hwMap.colorSensor.get("bottom sensor");
    imu = hwMap.get(BNO055IMU.class, "imu");
    cdim = hwMap.deviceInterfaceModule.get("dim");
    shootTouchSensor = hwMap.touchSensor.get("touch");
    leftMotor   = hwMap.dcMotor.get("left_drive");
    rightMotor  = hwMap.dcMotor.get("right_drive");
    rightShoot  = hwMap.dcMotor.get("right_shoot");
    collectorMotor = hwMap.dcMotor.get("collector motor");
    dispensingServo = hwMap.servo.get("servo");
    beaconColorSensor.setI2cAddress(I2cAddr.create8bit(0x44));
    colorSensor2.setI2cAddress(I2cAddr.create8bit(0x42));

    beaconColorSensor.enableLed(false);

    initMotors();
    initIMU();
}

    public void initTeleop(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        cdim = hwMap.deviceInterfaceModule.get("dim");
        shootTouchSensor = hwMap.touchSensor.get("touch");
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
        rightShoot  = hwMap.dcMotor.get("right_shoot");
        collectorMotor = hwMap.dcMotor.get("collector motor");
        dispensingServo = hwMap.servo.get("servo");

        initMotors();
    }

    private void initMotors() {
        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightShoot.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        collectorMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        rightShoot.setPower(0);
        collectorMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 200); //ToDo: original 1000 ms.  try smaller number to see if can get imu data quicker
    }

}

