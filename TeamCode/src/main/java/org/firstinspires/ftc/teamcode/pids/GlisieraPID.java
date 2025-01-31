package org.firstinspires.ftc.teamcode.pids;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@Config
@TeleOp
public class GlisieraPID extends OpMode {
    private PIDController gdrC, gstC;

    public static double gstP = 0.03, gdrP = 0.02, gstI = 0.03, gdrI = 0.03, gstD = 0, gdrD = 0;
    public static double gstF = 0.01, gdrF = 0.01;

    public static int target = 0;

    public static double saturation = 0.5;
    public static int gstOffset = 0, gdrOffset = 0;
    private final double ticks2deg = 384.5 / 360.0;

    private DcMotorEx gst, gdr;
    TouchSensor touchDR, touchST;

    @Override
    public void init() {
        gdrC = new PIDController(gdrP, gdrI, gdrD);
        gstC = new PIDController(gstP, gstI, gstD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gst = hardwareMap.get(DcMotorEx.class,"gst");

        gdr = hardwareMap.get(DcMotorEx.class,"gdr");
        gdr.setDirection(DcMotorSimple.Direction.REVERSE);

        touchDR = hardwareMap.get(TouchSensor.class, "touchDR");
        touchST = hardwareMap.get(TouchSensor.class,"touchST");
    }

    @Override
    public void loop() {
        gdr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gdrC.setPID(gdrP, gdrI, gdrD);
        gstC.setPID(gstP, gstI, gstD);
        int gdrPos = gdr.getCurrentPosition();
        int gstPos = gst.getCurrentPosition();

        double gstPID = gstC.calculate(gstPos, target+gstOffset);
        double gdrPID = gdrC.calculate(gdrPos, target+gdrOffset);
        double gstFF = Math.cos(Math.toRadians((target+gstOffset) / ticks2deg)) * gstF;
        double gdrFF = Math.cos(Math.toRadians((target+gdrOffset) / ticks2deg)) * gdrF;

        double gstPower = gstPID + gstFF;
        double gdrPower = gdrPID + gdrFF;

        if(gstPower >= saturation)
            gstPower = saturation;
        if(gstPower <= -saturation)
            gstPower = -saturation;
        if(gst.getCurrentPosition() <= 300 && !touchST.isPressed() && target == 0)
            gstPower = -0.3;

        if(gdrPower >= saturation)
            gdrPower = saturation;
        if(gdrPower <= -saturation)
            gdrPower = -saturation;
        if(gdr.getCurrentPosition() <= 300 && !touchDR.isPressed() && target == 0)
            gdrPower = -0.3;

        gst.setPower(gstPower);
        gdr.setPower(gstPower);


        if(touchDR.isPressed()) {
            gdr.setPower(0);
            gdr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(touchST.isPressed()) {
            gst.setPower(0);
            gst.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        telemetry.addData("target ", target);
        telemetry.addData("gstPos ", gstPos);
        telemetry.addData("gdrPos ", gdrPos);
        telemetry.addData("gstPower ", gstPower);
        telemetry.addData("gdrPower ", gdrPower);
        telemetry.update();
    }
}
