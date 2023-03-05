package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
    private AddressableLED LED;
    private AddressableLEDBuffer buffer;
    private int port;
    private int length;
    int[] pattern;

    public LED(int port, int length) {
        this.port = port;
        this.length = length;

        LED = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);

        LED.setLength(buffer.getLength());
        LED.setData(buffer);
        LED.start();

        clear();
    }

    public void setSolid(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        LED.setData(buffer);
    }

    public void setPatternSingleColor(int[] values) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, values[i], values[i], values[i]);
        }
        LED.setData(buffer);
    }

    public void setPatternRGB(int[][] values) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, values[i][0], values[i][1], values[i][2]);
        }
        LED.setData(buffer);
    }

    public void clear() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        LED.setData(buffer);
    }

    public int length() {
        return length;
    }
}