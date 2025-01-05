package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class Motor {

    private double lastSetPower = -1e7;
    private double setPowerTolerance = 0.05;
    private boolean reversed = false;

    private LynxModuleIntf pretendModule;

    public Motor(DcMotorEx motor){
        //pretendModule = new PretendLynxModule();
    }

    public void setReversed(boolean reversed){
        this.reversed = reversed;
    }

    public void setPower(double power){
        if(Math.abs(power - lastSetPower) > setPowerTolerance) return;

        lastSetPower  = Range.clip( power*(!reversed?1:-1), -1, 1);

        /*LynxCommand<LynxAck> command = new LynxSetMotorConstantPowerCommand(, , lastSetPower);

            try {
                command.send();
            }
            catch (InterruptedException | RuntimeException | LynxNackException ignored){}
        }*/
    }

}
