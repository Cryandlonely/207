#include "include/SerialManager.h"

#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <cstring>

static std::string FIVEG = "";
static std::string MESH = "";
int serial_port;

int sendData(int serial_port, const char* msg) {
  ssize_t msgdata = write(serial_port, msg, strlen(msg));
  sleep(0.01);
  if (msgdata < 0) {
    std::cerr << "发送失败" << std::endl;
    return 1;
  } else {
    // std::cout << "发送成功: " << msgdata << " 字节" << std::endl;
    return 0;
  }
}

int receiveData(char* buffer, size_t buffer_size) {
  if (serial_port < 0) {
    std::cerr << "串口未打开" << std::endl;
    return -1;
  }

  ssize_t bytes_read = read(serial_port, buffer, buffer_size - 1);

  if (bytes_read < 0) {
    std::cerr << "读取数据失败" << std::endl;
    return -1;
  }

  return bytes_read;
}

std::string readSerialInfo() {
  char buffer[1024];
  int n = receiveData(buffer, sizeof(buffer));
  if (n <= 0) {
    return std::string();
  }
  buffer[n] = '\0';
  return std::string(buffer, n);
}

void serialInit(std::string fiveG, std::string mesh) {
  FIVEG = fiveG;
  MESH = mesh;
  const char* port_name = "/dev/ttyUSB0";
  serial_port = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);

  if (serial_port < 0) {
    std::cerr << "无法打开串口: " << port_name << std::endl;
    return;
  }

  // 2. 配置串口参数
  struct termios tty;
  memset(&tty, 0, sizeof(tty));

  // 获取当前设置
  if (tcgetattr(serial_port, &tty) != 0) {
    std::cerr << "配置错误 (tcgetattr)" << std::endl;
    close(serial_port);
    serial_port = -1;
    return;
  }

  // 设置波特率 (115200)
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  // 常用配置 (8数据位，1停止位，无校验)
  tty.c_cflag &= ~PARENB;         // 禁用奇偶校验
  tty.c_cflag &= ~CSTOPB;         // 1停止位 (若需2停止位则置位)
  tty.c_cflag &= ~CSIZE;          // 清除数据位掩码
  tty.c_cflag |= CS8;             // 8数据位
  tty.c_cflag &= ~CRTSCTS;        // 禁用硬件流控
  tty.c_cflag |= CREAD | CLOCAL;  // 启用接收，忽略控制线

  // 原始输入模式 (禁用规范模式)
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~(ECHO | ECHOE | ECHONL);  // 禁用回显
  tty.c_lflag &= ~ISIG;                     // 禁用信号字符

  // 原始输出模式
  tty.c_oflag &= ~OPOST;  // 禁用特殊输出处理

  // 超时设置：立即返回（最小读取0字节）
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  // 保存配置
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    std::cerr << "配置错误 (tcsetattr)" << std::endl;
    close(serial_port);
    serial_port = -1;
    return;
  }

  // 初始化命令序列
  const char* init_commands[] = {"\n", "admin\n", "admin\n", "\n", "configure\n", "spanning-tree\n"};
  for (const char* cmd : init_commands) {
    sendData(serial_port, cmd);
  }
}

void fiveG(void) {
  //打开5G
  sendData(serial_port, ("int " + FIVEG + "\n").c_str());
  sendData(serial_port, "no shutdown\n");
  sendData(serial_port, "exit\n");

  sendData(serial_port, "int g6\n");
  sendData(serial_port, "no shutdown\n");
  sendData(serial_port, "exit\n");
  //关闭自组网
  sendData(serial_port, ("int " + MESH + "\n").c_str());
  sendData(serial_port, "shutdown\n");
  sendData(serial_port, "exit\n");
}

void mesh(void) {
  //打开自组网
  sendData(serial_port, ("int " + MESH + "\n").c_str());
  sendData(serial_port, "no shutdown\n");
  sendData(serial_port, "exit\n");
  //关闭5G
  sendData(serial_port, ("int " + FIVEG + "\n").c_str());
  sendData(serial_port, "shutdown\n");
  sendData(serial_port, "exit\n");

  sendData(serial_port, "int g6\n");
  sendData(serial_port, "shutdown\n");
  sendData(serial_port, "exit\n");
}
