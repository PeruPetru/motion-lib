package org.firstinspires.ftc.teamcode.util;

import java.io.FileWriter;
import java.io.IOException;

public class SimpleLogManager {
    public enum Type{
        CRITICAL,
        WARNING,
        INFORMATION
    }

    private static SimpleLogManager logManager = null;
    private FileWriter logWriter;

    private SimpleLogManager() {
        try {
            logWriter = new FileWriter("Logs");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static SimpleLogManager getInstance(){
        if(logManager == null)
            logManager = new SimpleLogManager();

        return logManager;
    }

    public void addLog(Type type, String identifier, String logMessage){
        try {
            logWriter.write("x");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
}
