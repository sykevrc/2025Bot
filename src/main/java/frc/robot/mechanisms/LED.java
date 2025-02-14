package frc.robot.mechanisms;

import edu.wpi.first.wpilibj.PWM;


public class LED {
    private int channel = 0;
    private PWM pwm = null;
    private LEDStatus ledStatus = LEDStatus.ready;

    private static final int STROBE_WHITE = 1475;
    private static final int STROBE_BLUE = 1455;
    private static final int LIGHT_CHASE_RED = 1345;
    private static final int HEARTBEAT_RED = 1375;
    private static final int LARSON_SCANNER_RED = 1325;

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

    public void setStatus(LEDStatus ledStatus) {
        this.ledStatus = ledStatus;

        switch(ledStatus) {
            case ready:
                //System.out.println("setting led to ready");
                pwm.setPulseTimeMicroseconds(1855);
                break;
            case problem:
                //System.out.println("setting led to problem");
                pwm.setPulseTimeMicroseconds(1795);
                break;
            case targetAquired:
                //System.out.println("setting led to targetAquired");
                pwm.setPulseTimeMicroseconds(STROBE_WHITE);
                break;
            case targetSearching:
                //System.out.println("setting led to targetSearching");
                pwm.setPulseTimeMicroseconds(LARSON_SCANNER_RED);
                break;
            case hasCoral:
                pwm.setPulseTimeMicroseconds(STROBE_BLUE);
                break;
            case hasAlgae:
                pwm.setPulseTimeMicroseconds(STROBE_WHITE);
                break;
            default:
                break;
        }
    }
}
