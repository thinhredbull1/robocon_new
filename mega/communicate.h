
struct CommandPair {
  String sendCommand;
  String receiveCommand;
  bool flag_send;
  bool flag_rec;
  void (*callback)();
};
void callback1() {
  Serial.println("Received command 1!");
}
void callback2() {
  Serial.println("Received command 2!");
}
enum signal {
  START = 0,
  RESET = 1,
  XYZ = 2
};
CommandPair commands[] = {
  { "SET:START;", "PI:START", false,false, callback1 },
  { "SET:BALL1;", "PI:BALL1", false,false, callback2 },
};
void send(int index) {
  Serial.print("Sending command: ");
  Serial.println(commands[index].sendCommand);
  commands[index].flag_send = true;
  Serial.println("...");  // Thêm các lệnh gửi ở đây (ví dụ: Serial.println(command);)
}
void serial_handle() {
  String rec_str;
  static unsigned long time_retry = millis();
  bool send_signal=0;
  if (Serial.available()) {
    rec_str = Serial.readStringUntil(';');
        for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
    
        if (rec_str.equals(commands[i].receiveCommand))
        {
            commands[i].flag_rec=1;
        }
        Serial.println(rec_str);
    }
  }
  for (int i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
    if (commands[i].flag_send == true) {
      if (commands[i].flag_rec) {
        commands[i].flag_send = false;
        commands[i].flag_rec=false;
        commands[i].callback();
      } else if (millis() - time_retry > 1000) {
        commands[i].flag_send = false;
        Serial.println("time out");
      }
      send_signal=1;
    }
    if(commands[i].flag_rec)
    {
        send(i);
        commands[i].callback();
        commands[i].flag_rec=false;
    } 
  }
  if(!send_signal)time_retry=millis();
}