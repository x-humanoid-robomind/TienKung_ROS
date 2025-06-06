#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <iomanip>

class SerialPort
{
public:
    SerialPort(const std::string &port, speed_t baud_rate, int bytesize, int parity, int stopbits)
    {
        m_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (m_fd == -1)
        {
            throw std::runtime_error("Failed to open serial port: " + port);
        }

        struct termios options;
        tcgetattr(m_fd, &options);

        options.c_cflag = baud_rate | bytesize | parity | stopbits | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;
        tcflush(m_fd, TCIFLUSH);
        tcsetattr(m_fd, TCSANOW, &options);
    }

    int get_fd()
    {
        return m_fd;
    }
    ~SerialPort()
    {
        if (m_fd != -1)
        {
            close(m_fd);
        }
    }

    int read_data(char *buffer, size_t buffer_size)
    {
        return ::read(m_fd, buffer, buffer_size);
    }

private:
    int m_fd = -1;
};

speed_t get_custom_baud(int baud_rate)
{
    speed_t custom_baud;
    switch (baud_rate)
    {
    case 50:
        custom_baud = B50;
        break;
    case 75:
        custom_baud = B75;
        break;
    // Add more cases for other standard baud rates
    default:
        custom_baud = B0;
        custom_baud = (speed_t)baud_rate;
        break;
    }
    return custom_baud;
}

int main()
{
    int baud_rate = 100000;
    speed_t custom_baud = get_custom_baud(baud_rate);

    SerialPort serial_port("/dev/ttyUSB0", custom_baud, CS8, PARENB, CSTOPB);
    const size_t buffer_size = 4096;
    char readData[buffer_size];
    int channel[12];
    std::vector<unsigned char> buffer;

    int mode = 0;
    while (true)
    {
        int bytes_read = serial_port.read_data(readData, buffer_size);
        std::vector<unsigned char> readDataVector(readData, readData + sizeof(readData));
        buffer.insert(buffer.end(), readDataVector.begin(), readDataVector.end());

        if (buffer.size() > 0) // 首先需要超过0
        {
            for (int i = 0; i < buffer.size(); ++i)
            {
                if (static_cast<unsigned char>(buffer[i]) == 0x0F) // 首找到头
                {
                    if (buffer.size() >= 25 + i) // 找到头之后还够25
                    {
                        // std::cout << "Received data with header 0x0F++++++++" << std::endl;
                        if (static_cast<unsigned char>(buffer[24 + i]) == 0x00)
                        {
                            // std::cout << "Received data with header 0x00~--------" << std::endl;

                            int start_idx = i;
                            channel[0] = (buffer[start_idx + 1] | (buffer[start_idx + 2] << 8) | (buffer[start_idx + 3] << 16)) & 0x0007FF;
                            channel[1] = ((buffer[start_idx + 2] >> 3) | (buffer[start_idx + 3] << 8 >> 3) | (buffer[start_idx + 4] << 16 >> 3)) & 0x0007FF;
                            channel[2] = (buffer[start_idx + 3] >> 6 | (buffer[start_idx + 4] << 8 >> 6) | (buffer[start_idx + 5] << 16 >> 6)) & 0x0007FF;
                            channel[3] = (buffer[start_idx + 5] >> 1 | (buffer[start_idx + 6] << 8 >> 1) | (buffer[start_idx + 7] << 16 >> 1)) & 0x0007FF;
                            channel[4] = (buffer[start_idx + 6] >> 4 | (buffer[start_idx + 7] << 8 >> 4) | (buffer[start_idx + 8] << 16 >> 4)) & 0x0007FF;
                            channel[5] = (buffer[start_idx + 7] >> 7 | (buffer[start_idx + 8] << 8 >> 7) | (buffer[start_idx + 9] << 16 >> 7)) & 0x0007FF;
                            channel[6] = (buffer[start_idx + 9] >> 2 | (buffer[start_idx + 10] << 8 >> 2) | (buffer[start_idx + 11] << 16 >> 2)) & 0x0007FF;
                            channel[7] = (buffer[start_idx + 10] >> 5 | (buffer[start_idx + 11] << 8 >> 5) | (buffer[start_idx + 12] << 16 >> 5)) & 0x0007FF;
                            channel[8] = (buffer[start_idx + 11] >> 8 | (buffer[start_idx + 12] << 8 >> 8) | (buffer[start_idx + 13] << 16 >> 8)) & 0x0007FF;
                            channel[9] = (buffer[start_idx + 13] >> 3 | (buffer[start_idx + 14] << 8 >> 3) | (buffer[start_idx + 15] << 16 >> 3)) & 0x0007FF;
                            channel[10] = (buffer[start_idx + 14] >> 6 | (buffer[start_idx + 15] << 8 >> 6) | (buffer[start_idx + 16] << 16 >> 6)) & 0x0007FF;
                            channel[11] = (buffer[start_idx + 16] >> 1 | (buffer[start_idx + 17] << 8 >> 1) | (buffer[start_idx + 18] << 16 >> 1)) & 0x0007FF;
                            std::cout << "channel ";
                            for (int i = 0; i < 12; ++i)
                            {
                                std::cout << " " << i << " : " << channel[i];
                            }
                            // 您可以在这里处理接收到的数据
                            std::cout << std::endl;
                        }

                        mode = 1; // 不做处理
                        // std::cout << "not 00 end" << std::endl;
                        buffer.erase(buffer.begin(), buffer.end() + 25 + i);
                    }
                    else
                    {
                        mode = 1; // 不做处理
                    }
                }
            }
            if (mode == 0)
            {
                buffer.erase(buffer.begin(), buffer.end());
            }
            // else
            // {
            //     // std::cout << "Received data without 0x0F header" << std::endl;
            //     std::cout << "Received " << bytes_read << " bytes: ";
            //     std::cout << std::hex << std::setfill('0');
            //     for (int i = 0; i < bytes_read; i++)
            //     {
            //         std::cout << "0x" << std::setw(2) << static_cast<int>(static_cast<unsigned char>(buffer[i])) << " ";
            //     }
            //     std::cout << std::endl;
            //     tcflush(serial_port.get_fd(), TCIFLUSH);
            // }
        }
        else
        {
            // std::cerr << "Failed to read data from serial port" << std::endl;
            // 您可以添加一些延迟,防止 CPU 占用过高
            // 清空输入缓冲区
            // tcflush(serial_port.get_fd(), TCIFLUSH);
            // 您可以添加一些延迟,防止 CPU 占用过高
            usleep(100); // 100 毫秒
        }
    }

    return 0;
}