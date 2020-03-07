/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
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
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Test1", group="Test")

public class Test1 extends LinearOpMode {

    /* Declare OpMode members. */
    pushbotHardware robot = new pushbotHardware();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "ARoDiMD/////AAABme/gmX+bLkNYtESiRDX+TRaF+wryWg1vKuZYngn0EyQx+4Gp76vfA6zrEqgLbkgUFocmm2acaqTAh9o4lORYo8PlGXVpnvY4oeLJsMWmr4ro4yznHjGYhs3gIqenqlgwYt8WmkyKciZ9PTcbhS1nxSOWHGaHu+SEQuwqMBWs64SJNCE5s+pzg/A4WUt4cA3lGHHISWfxX4ug37IG5Nl3xfn/DdcYcVCbzKgnh+pszAvTe+lzJMNd7nbMNyxhGalZDGAnrSho/OMT+F9qcJYQ/NskHdTMjx9nblrXfaFq1LV/BNL9foXi9Vhe/ld+caWaebKgkNGFTdDD1ShoCKp3pkffoqDd5fhCG1AtmQZd6iLT";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;

    private boolean targetVisible = false;


    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 100/mmPerInch ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.4;     // Nominal half speed for better accuracy.
    static final double     STRAFE_SPEED            = 1;

    static final double     HEADING_THRESHOLD       = 0.3;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.1;     // Larger is more responsive, but also less stable

    private double prevAngle = 0;
    private int stone = 3;
    private double z;

    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        //vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vparameters.vuforiaLicenseKey = "ARoDiMD/////AAABme/gmX+bLkNYtESiRDX+TRaF+wryWg1vKuZYngn0EyQx+4Gp76vfA6zrEqgLbkgUFocmm2acaqTAh9o4lORYo8PlGXVpnvY4oeLJsMWmr4ro4yznHjGYhs3gIqenqlgwYt8WmkyKciZ9PTcbhS1nxSOWHGaHu+SEQuwqMBWs64SJNCE5s+pzg/A4WUt4cA3lGHHISWfxX4ug37IG5Nl3xfn/DdcYcVCbzKgnh+pszAvTe+lzJMNd7nbMNyxhGalZDGAnrSho/OMT+F9qcJYQ/NskHdTMjx9nblrXfaFq1LV/BNL9foXi9Vhe/ld+caWaebKgkNGFTdDD1ShoCKp3pkffoqDd5fhCG1AtmQZd6iLT";
        vparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.getInstance().createVuforia(vparameters);
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); // alt + enter for shortcut to import smthn
        parameters.mode = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        targetsSkyStone.activate();

        // Set up our telemetry dashboard
        //composeTelemetry();
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        waitForStart();  //make sure gyro is calibrated before pressing start button

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            //telemetry.addData(">", "Robot Heading = %d", imu.getAngularOrientation());
            //telemetry.update();
        }
        //TOP
        while (opModeIsActive()) {
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            // Put a hold after each turn

            telemetry.update();

            gyroDrive(DRIVE_SPEED, 14,0);
            gyroTurn2(TURN_SPEED, 90);
            gyroHold2(TURN_SPEED,90, 0.1);
            armsDown();

            ElapsedTime scanTimer = new ElapsedTime();
            // keep looping while we have time remaining.
            scanTimer.reset();
            while (!targetVisible || scanTimer.seconds() <= 2) {
                // check all the trackable targets to see which one (if any) is visible.
                telemetry.addData("Timer: ", scanTimer.time());
                telemetry.addData("Scanning: ", !targetVisible || scanTimer.time() >= 3);
                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        targetVisible = true;


                        // getUpdatedRobotLocation() will return null if no new information is available since
                        // the last time that call was made, or if the trackable is not currently visible.
                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }
                        break;
                    }
                }
                telemetry.update();
            }
            if (targetVisible) {
                telemetry.addData("Target Visible", targetVisible);
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                if(translation.get(1)/mmPerInch < 0)
                    stone = 2;
                else if (translation.get(1)/mmPerInch < 15)
                    stone = 1;
            } else {
                telemetry.addData("Target Visible", "none");
            }
            telemetry.addData("Stone ", stone);
            telemetry.update();

            switch(stone) {
                case 1:
                    gyroStrafe(STRAFE_SPEED, -16, 0);
                    gyroHold2(TURN_SPEED,90, 0.5);
                    grab1();
                    armsUp();
                    gyroStrafe(STRAFE_SPEED, 7, 0);
                    gyroHold(TURN_SPEED,90,0.5);
                    gyroDrive(DRIVE_SPEED, 72, 0);
                    gyroHold(TURN_SPEED,90,0.5);
                    gyroStrafe(STRAFE_SPEED, -7, 0);
                    armsDown();
                    release();
                    gyroHold2(TURN_SPEED,90,0.5);
                    gyroStrafe(STRAFE_SPEED, 7, 0);
                    gyroHold(TURN_SPEED,90,0.5);
                    gyroDrive(DRIVE_SPEED, -96, 0);
                    gyroHold(TURN_SPEED,90,0.5);
                    gyroStrafe(STRAFE_SPEED, -7, 0);
                    gyroHold2(TURN_SPEED,90,0.5);
                    grab1();
                    armsUp();
                    gyroStrafe(STRAFE_SPEED, 7, 0);
                    gyroDrive(DRIVE_SPEED, -12, 0);
                    gyroDrive(DRIVE_SPEED, 116, 0);
                    gyroHold(TURN_SPEED,90,0.5);
                    break;
                case 2:
                    gyroStrafe(STRAFE_SPEED, -13, 0);
                    gyroTurn(TURN_SPEED,90);
                    grab1();
                    armsUp();
                    gyroStrafe(STRAFE_SPEED, 3, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    gyroDrive(DRIVE_SPEED, 84, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    gyroStrafe(0.3, -3, 0);
                    armsDown();
                    release();
                    gyroStrafe(STRAFE_SPEED, 3, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    gyroDrive(DRIVE_SPEED, -108, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    gyroStrafe(0.3, -3, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    grab1();
                    armsUp();
                    gyroStrafe(STRAFE_SPEED, 3, 0);
                    gyroDrive(DRIVE_SPEED, 116, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    break;
                case 3:
                    gyroDrive(DRIVE_SPEED, -4,0);
                    gyroHold(0,0,1);
                    gyroStrafe(STRAFE_SPEED, -13, 0);
                    gyroTurn(TURN_SPEED,90);
                    grab1();
                    armsUp();
                    gyroStrafe(STRAFE_SPEED, 3, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    gyroDrive(DRIVE_SPEED, 88, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    gyroStrafe(0.3, -3, 0);
                    armsDown();
                    release();
                    gyroStrafe(STRAFE_SPEED, 3, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    gyroDrive(DRIVE_SPEED, -112, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    gyroStrafe(0.3, -3, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    grab1();
                    armsUp();
                    gyroStrafe(STRAFE_SPEED, 3, 0);
                    gyroDrive(DRIVE_SPEED, 120, 0);
                    gyroHold(TURN_SPEED,90,0.1);
                    break;
            }
            gyroStrafe(STRAFE_SPEED, -7,90);
            release();
            gyroTurn2(TURN_SPEED, 0);
            //gyroHold(TURN_SPEED,0,0.1);
            gyroDrive(0.4, 3,0);
            gyroDrive(0.2, 3,0);
            closeFoundation();
            gyroHold(0,0,1);
            gyroDrive(1, -24, 0);
            gyroTurn2(TURN_SPEED,90);
            openFoundation();
            gyroTurn(TURN_SPEED, 180);
            extendMTape();
            gyroHold(0,180,10);





            // Disable Tracking when we are done;
            targetsSkyStone.deactivate();



            telemetry.addData("Path", "Complete");
            telemetry.update();

        }
    }

    private void grab1(){
        robot.grab1.setPosition(0);
    }

    private void grab2(){
        robot.grab2.setPosition(1);
    }

    private void release(){
        robot.grab1.setPosition(1);
        robot.grab2.setPosition(0);
    }

    private void armsDown(){
        robot.arm1.setPosition(0);
        robot.arm2.setPosition(1);
        robot.grab1.setPosition(1);
        robot.grab2.setPosition(0);
    }

    private void armsUp(){
        robot.arm1.setPosition(1);
        robot.arm2.setPosition(0);
    }

    private void extendMTape(){
        robot.mTape.setPower(-1);
        sleep(2000);
        robot.mTape.setPower(0);
    }

    private void closeFoundation(){
        robot.clasp1.setPosition(1);
        robot.clasp2.setPosition(0);
    }
    private void openFoundation(){
        robot.clasp1.setPosition(0);
        robot.clasp2.setPosition(1);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vparameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vparameters.vuforiaLicenseKey = "ARoDiMD/////AAABme/gmX+bLkNYtESiRDX+TRaF+wryWg1vKuZYngn0EyQx+4Gp76vfA6zrEqgLbkgUFocmm2acaqTAh9o4lORYo8PlGXVpnvY4oeLJsMWmr4ro4yznHjGYhs3gIqenqlgwYt8WmkyKciZ9PTcbhS1nxSOWHGaHu+SEQuwqMBWs64SJNCE5s+pzg/A4WUt4cA3lGHHISWfxX4ug37IG5Nl3xfn/DdcYcVCbzKgnh+pszAvTe+lzJMNd7nbMNyxhGalZDGAnrSho/OMT+F9qcJYQ/NskHdTMjx9nblrXfaFq1LV/BNL9foXi9Vhe/ld+caWaebKgkNGFTdDD1ShoCKp3pkffoqDd5fhCG1AtmQZd6iLT";
        vparameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.getInstance().createVuforia(vparameters);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(vparameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            robot.leftFront.setPower(speed*0.2);
            robot.rightFront.setPower(speed*0.2);
            robot.leftBack.setPower(speed*0.2);
            robot.rightBack.setPower(speed*0.2);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getDriveSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) {
                    steer *= -1.0;
                }

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                while ( (robot.leftBack.getCurrentPosition() >= robot.leftBack.getTargetPosition()+20 && robot.leftBack.getCurrentPosition() <= robot.leftBack.getTargetPosition()-20) &&
                        (robot.leftFront.getCurrentPosition() >= robot.leftFront.getTargetPosition()+20 && robot.leftFront.getCurrentPosition() <= robot.leftFront.getTargetPosition()-20) &&
                        (robot.rightBack.getCurrentPosition() >= robot.rightBack.getTargetPosition()+20 && robot.rightBack.getCurrentPosition() <= robot.rightBack.getTargetPosition()-20) &&
                        (robot.rightFront.getCurrentPosition() >= robot.rightFront.getTargetPosition()+20 && robot.rightFront.getCurrentPosition() <= robot.rightFront.getTargetPosition()-20)) {
                    robot.leftFront.setPower(leftSpeed);
                    robot.leftBack.setPower(leftSpeed);
                    robot.rightFront.setPower(rightSpeed);
                    robot.rightBack.setPower(rightSpeed);
                }
                // Display drive status for the driver.

                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                //telemetry.addData("Target",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget,  newLeftBackTarget,  newRightBackTarget);
                //telemetry.addData("Actual",  "%7d:%7d",      robot.leftFront.getCurrentPosition(),
                //robot.leftBack.getCurrentPosition(),
                //robot.rightFront.getCurrentPosition(),
                //robot.rightBack.getCurrentPosition());
                telemetry.addData("Left Speed",   "%5.2f",  leftSpeed);
                telemetry.addData("Right Speed",   "%5.2f",  rightSpeed);
                telemetry.addData("Angle: ", intZ());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("Current Angle: ", intZ());
            telemetry.addData("Target Angle : ", intZ());
            telemetry.update();
        }
    }

    public void gyroTurn2 (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading2(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.addData("Current Angle: ", intZ());
            telemetry.addData("Target Angle : ", intZ());
            telemetry.update();
        }
    }



    public void gyroStrafe ( double speed,
                             double distance,
                             double angle) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  frontLeftSpeed;
        double  backLeftSpeed;
        double  frontRightSpeed;
        double  backRightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFront.getCurrentPosition() - moveCounts;
            newLeftBackTarget = robot.leftBack.getCurrentPosition() - moveCounts;
            newRightBackTarget = robot.rightBack.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(speed);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy())) {

                error = getError(angle);
                steer = getDriveSteer(error, P_DRIVE_COEFF);

                if (distance < 0) {
                    steer *= -1.0;
                }

                frontLeftSpeed = speed - steer;
                backLeftSpeed  = speed - steer;
                frontRightSpeed = speed + steer;
                backRightSpeed = speed + steer;


                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.max(Math.abs(backLeftSpeed), Math.abs(backRightSpeed)) , Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed)));
                if (max > 1.0)
                {

                    backLeftSpeed /= max;
                    backRightSpeed /= max;
                    frontLeftSpeed /= max;
                    frontRightSpeed /= max;
                }

                if ( (robot.leftBack.getCurrentPosition() >= robot.leftBack.getTargetPosition()+20 && robot.leftBack.getCurrentPosition() <= robot.leftBack.getTargetPosition()-20) &&
                        (robot.leftFront.getCurrentPosition() >= robot.leftFront.getTargetPosition()+20 && robot.leftFront.getCurrentPosition() <= robot.leftFront.getTargetPosition()-20) &&
                        (robot.rightBack.getCurrentPosition() >= robot.rightBack.getTargetPosition()+20 && robot.rightBack.getCurrentPosition() <= robot.rightBack.getTargetPosition()-20) &&
                        (robot.rightFront.getCurrentPosition() >= robot.rightFront.getTargetPosition()+20 && robot.rightFront.getCurrentPosition() <= robot.rightFront.getTargetPosition()-20)) {
                    robot.leftFront.setPower(frontLeftSpeed);
                    robot.leftBack.setPower(backLeftSpeed);
                    robot.rightFront.setPower(frontRightSpeed);
                    robot.rightBack.setPower(backRightSpeed);
                }
                // Display drive status for the driver.
                telemetry.addData("Target",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget,  newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftFront.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.addData("Right Speed: ",   "%5.2f:%5.2f", frontRightSpeed , backRightSpeed);
                telemetry.addData("Left Speed : ",   "%5.2f:%5.2f", frontLeftSpeed , backLeftSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.seconds() <= holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void gyroHold2( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.seconds() <= holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading2(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getTurnSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    boolean onHeading2(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= 1) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getTurnSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFront.setPower(leftSpeed);
        robot.leftBack.setPower(leftSpeed);
        robot.rightFront.setPower(rightSpeed);
        robot.rightBack.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - intZ();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double intZ(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (prevAngle - angles.firstAngle);
    }
    public void resetIntZ(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        prevAngle = angles.firstAngle;

    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getDriveSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, 0, 0.5);
    }
    public double getTurnSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -0.75, 0.75);
    }

}
