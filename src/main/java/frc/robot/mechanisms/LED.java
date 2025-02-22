package frc.robot.mechanisms;

import edu.wpi.first.wpilibj.PWM;


public class LED {
    private int channel = 0;
    private PWM pwm = null;
    private LEDStatus ledStatus = LEDStatus.ready;

    private static final int STROBE_WHITE = 1475;
    private static final int STROBE_BLUE = 1455;
    private static final int STROBE_GOLD = 1465;
    private static final int STROBE_RED = 1445;
    private static final int STROBE_COLOR1 = 1575;
    private static final int STROBE_COLOR2 = 1675;

    private static final int HEARTBEAT_RED = 1375;

    private static final int LARSON_SCANNER_RED = 1325;
    private static final int LARSON_SCANNER_GREY = 1335;
    private static final int LARSON_SCANNER_COLOR1 = 1495;
    private static final int LARSON_SCANNER_COLOR2 = 1595;

    private static final int SOLID_RED = 1805;
    private static final int SOLID_DARK_RED = 1795;
    private static final int SOLID_HOT_PINK = 1785;
    private static final int SOLID_DARK_GREEN = 1875;
    private static final int SOLID_ORANGE= 1825;

    private static final int LIGHT_CHASE_RED = 1345;
    private static final int LIGHT_CHASE_COLOR1 = 1505;
    private static final int LIGHT_CHASE_COLOR2 = 1605;
    

    public static enum LEDStatus {
        ready,
        problem,
        targetAquired,
        targetSearching,
        hasCoral,
        hasAlgae
    }

    public LED(int channel) {
        this.channel = channel;

        pwm = new PWM(channel);
        setStatus(LEDStatus.ready);
    }

    public LEDStatus getStatus() {
        return this.ledStatus;
    }

    public void setStatus(LEDStatus ledStatus) {
        this.ledStatus = ledStatus;

        switch(ledStatus) {
            case ready:
                //System.out.println("setting led to ready");
                pwm.setPulseTimeMicroseconds(LARSON_SCANNER_RED);
                break;
            case problem:
                //System.out.println("setting led to problem");
                pwm.setPulseTimeMicroseconds(LARSON_SCANNER_RED);
                break;
            case targetAquired:
                //System.out.println("setting led to targetAquired");
                pwm.setPulseTimeMicroseconds(STROBE_GOLD);
                break;
            case targetSearching:
                //System.out.println("setting led to targetSearching");
                pwm.setPulseTimeMicroseconds(LARSON_SCANNER_RED);
                break;
            case hasCoral:
                pwm.setPulseTimeMicroseconds(STROBE_WHITE);
                break;
            case hasAlgae:
                pwm.setPulseTimeMicroseconds(STROBE_BLUE);
                break;
            default:
                break;
        }
    }
}
