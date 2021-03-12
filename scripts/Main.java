
import java.io.*;
import java.net.*;

class Main {
    public static void main(String[] args) {
        SocketServer server;
        String ip = "localhost";
        int port = 5038;
        try {
            server = new Socket(ip, port);
            var out    = new DataOutputStream(server.getOutputStream());
            for (int i = 0; i < 10; i++) {
                out.writeBytes("a");
            }
            out.close();
            server.close();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            return;
        }
    }
}