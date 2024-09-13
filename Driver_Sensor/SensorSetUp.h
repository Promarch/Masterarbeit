// No headers for the moment


int serial_port_declare;

class myBotaForceTorqueSensorComm : public BotaForceTorqueSensorComm
{
  public:
  int serialReadBytes(uint8_t* data, size_t len) override {
    return read(serial_port_declare, data, len);
  }
  int serialAvailable() override {
    int bytes;
    ioctl(serial_port_declare, FIONREAD, &bytes);
    return bytes;
  }
};
// Set Up of the serial connector using termios
void set_up_serial(int _serial_port);
// Initialisation and preliminary check of the sensor
void check_sensor(myBotaForceTorqueSensorComm _sensor);

