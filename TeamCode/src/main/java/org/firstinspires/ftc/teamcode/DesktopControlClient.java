package org.firstinspires.ftc.teamcode;
/*
    this is not supposed to be in org.firstinspires.ftc.teamcode. DON'T MOVE IT
    it's a regular java class with a regular main method, not an opmode
    read my doc for more info: https://docs.google.com/document/d/1bxNabZWSuYS4XG__-KeR-mj_PQDcAdDYHsHEZVNLnCc/edit?tab=t.0

    copy and paste the following into the terminal in Android Studio to run this class:
        cd TeamCode/src/main/java
        javac DesktopControlClient.java
        java DesktopControlClient
*/

import java.io.*;
import java.net.*;
import java.util.Scanner;

public class DesktopControlClient {
    private static Socket socket;
    private static Scanner read;
    private static DataOutputStream out;
    private static DataInputStream in;

    private static final String IP = "192.168.43.1";

    public static void main(String[] args) throws IOException, UnknownHostException {
        // connect to the phone
        socket = new Socket(IP, 1234); // 1234 is the port
        System.out.println("Connected to the phone!");

        // init streams and stuff
        read = new Scanner(System.in);
        out = new DataOutputStream(socket.getOutputStream());
        in = new DataInputStream(socket.getInputStream());

        while (true) {
            String output = read.nextLine();

            out.writeUTF(output);
            out.flush();

            String input = in.readUTF();
            System.out.println(input);
        }
    }
}
