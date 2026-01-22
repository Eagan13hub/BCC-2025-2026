package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
// import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name="BCC TeleOp")

public class BCTMainV1 extends LinearOpMode {
    // initHardware////////
    // initMotors
    DcMotor m0;
    DcMotor m1;
    DcMotor m2;
    DcMotor m3;
    DcMotor m6;
    // initImu
    public DcMotorEx shooter;
    private static final int REV_HD_MOTOR_TICKS_PER_ROTATION = 28;
    private static final double SHOOTER_MAX_RPM = 5200;
    double shooterTargetVelocity = 1300; // 1300 prior lake value
    // 5500 / 60 = rotations per second

    double longRangeVelocity =  1700;
    public double  rpmToTicksPerSecond(double rpm) {
        return rpm / 60 * REV_HD_MOTOR_TICKS_PER_ROTATION;
    }

    public double ticksPerSecondToRPM(double tps) {
        return tps / REV_HD_MOTOR_TICKS_PER_ROTATION * 60;
    }


    @Override
    public void runOpMode() {
        
        // initPaths////////
        // setupMotors
        m0 = hardwareMap.get(DcMotor.class, "m0"); // driveMotor0
        m1 = hardwareMap.get(DcMotor.class, "m1"); // driveMotor1
        m2 = hardwareMap.get(DcMotor.class, "m2"); // driveMotor2
        m3 = hardwareMap.get(DcMotor.class, "m3"); // driveMotor3
        m6 = hardwareMap.get(DcMotor.class, "m6"); // loader motor
        shooter = hardwareMap.get(DcMotorEx.class, "m7"); //lancher moter
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Assign motors to more friendly name/
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // initHardwareParams////////
        // initDriveMotors
        
        m0.setDirection(DcMotor.Direction.REVERSE); // front left
        m1.setDirection(DcMotor.Direction.REVERSE); // back left
        m2.setDirection(DcMotor.Direction.FORWARD); // front right
        m3.setDirection(DcMotor.Direction.FORWARD); //  back right
        // m0.setDirection(DcMotor.Direction.FORWARD);
        // m1.setDirection(DcMotor.Direction.FORWARD);
        // m2.setDirection(DcMotor.Direction.FORWARD);
        // m3.setDirection(DcMotor.Direction.FORWARD);
        
        m6.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        m0.setPower(0);
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        
        
        // defVars
        float speedMultiplyer = 1;
        double gyro = 0;

        double loaderTargetPower = 0.0;

        // mainLoop////////
        while (opModeIsActive()) {

            PIDFCoefficients currentPIDFCoefficients = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            if (currentPIDFCoefficients.p != X25Config.SHOOTER_PID.p ||
                    currentPIDFCoefficients.i != X25Config.SHOOTER_PID.i ||
                    currentPIDFCoefficients.d != X25Config.SHOOTER_PID.d ||
                    currentPIDFCoefficients.f != X25Config.SHOOTER_PID.f){
                shooter.setVelocityPIDFCoefficients(X25Config.SHOOTER_PID.p, X25Config.SHOOTER_PID.i, X25Config.SHOOTER_PID.d, X25Config.SHOOTER_PID.f);
            }
            
            // updateVariables////////
            // updateControlVars
            
            // intakeExtension = gamepad1Trigger;

            // handleDriveMotors////////
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // This is test code:
            //  
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            if (gamepad1.right_bumper)
            {
                m0.setPower(frontLeftPower/5);
                m2.setPower(frontRightPower/5);
                m1.setPower(backLeftPower/5);
                m3.setPower(backRightPower/5);
            } else {
                m0.setPower(frontLeftPower);
                m2.setPower(frontRightPower);
                m1.setPower(backLeftPower);
                m3.setPower(backRightPower);
            }
            

            // Show the elapsed game time and wheel power.
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        
            
        
        
            if (gamepad1.a)
            {
                if (shooter.getVelocity() > shooterTargetVelocity) {
                    loaderTargetPower = 1.0;
                } else {
                   loaderTargetPower = 0.0; 
                }
            } else {    // if a is not pressed
                loaderTargetPower = 0.0;
            }
            
            m6.setPower(loaderTargetPower);
            
            
            /*if (gamepad1.y) {
                shooter.setVelocity(shooterTargetVelocity);
            }*/


            shooter.setVelocity(shooterTargetVelocity);


            if (gamepad1.b){
                shooter.setVelocity(0);
            }
            if (gamepad1.x){
                shooter.setVelocity(longRangeVelocity);
            }
            // telemetry
            telemetry.addData("shooterTargetVelocityRPM", ticksPerSecondToRPM(shooterTargetVelocity));
            telemetry.addData("shooterVelocityRPM", ticksPerSecondToRPM(shooter.getVelocity()));
            telemetry.update();

            telemetry.addData("yaw", gyro);
            telemetry.addData("loaderTargetPower:", loaderTargetPower);
            telemetry.addData("launcherVelocity:", shooter.getVelocity());
            telemetry.update();
        }
        
    }}
