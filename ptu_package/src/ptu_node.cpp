#include "ptu_package/ptu_node.hpp" // cabecera con definicion de la clase PTUNode
// acceder al puerto serial y manejar mensajes
#include "std_msgs/msg/string.hpp" // manejar mensajes de ros
#include <unistd.h> // acceso a llamadas al sistema operativo
#include <fcntl.h> // control de archivos en sistemas POSIX
#include <cstring> // manipulacion de cadenas

PTUNode::PTUNode() : Node("PTU_node") {
    std::string port = this->declare_parameter("serial_port", "/dev/ttyUSB0"); // asignar puerto serial

    if (open_serial(port)) {
        RCLCPP_INFO(this->get_logger(), "Conectado a PTU en %s", port.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el puerto: %s", port.c_str());
    }
    
    // suscripcion a un topico
    command_subscriber_ = this->create_subscription<std_msgs::msg::String>("ptu_cmd", 10, std::bind(&PTUNode::command_callback, this, std::placeholders::_1));

}
// cerrar descriptor de archivo asociado a puerto serial si esta abierto
PTUNode::~PTUNode() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
    }
}

// abrir el puerto en modo lectura y escritura, no terminal de control, espera que las escrituras se completen -> sincrono
bool PTUNode::open_serial(const std::string &device) {
    serial_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) return false;

    struct termios tty; // estructura para configurar el puerto
    memset(&tty, 0, sizeof tty); // pone todos los bytes de la estructura tty en cero
    if (tcgetattr(serial_fd_, &tty) != 0) return false;

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag = CS8 | CLOCAL | CREAD;                 // 8 bits, sin modem control, lectura activa
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);        // sin paridad, 1 stop bit, sin RTS/CTS
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);             // sin XON/XOFF
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                     INLCR | IGNCR | ICRNL);            // sin conversión de entrada
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;

    tcflush(serial_fd_, TCIOFLUSH);
    return tcsetattr(serial_fd_, TCSANOW, &tty) == 0;
}

void PTUNode::send_command(const std::string &cmd) {
    std::string full_cmd = cmd + '\r';

    // Mostrar los bytes que se enviarán (debug en hex)
    std::ostringstream debug;
    for (unsigned char c : full_cmd) {
        debug << std::hex << std::showbase << (int)c << " ";
    }

    RCLCPP_INFO(this->get_logger(),
        "Comando recibido: '%s'\nBytes enviados: %s",
        cmd.c_str(), debug.str().c_str()
    );

    // Limpiar buffer de entrada antes de enviar
    tcflush(serial_fd_, TCIFLUSH);

    // Enviar carácter por carácter con pequeña pausa para emular 'screen'
    for (char c : full_cmd) {
        write(serial_fd_, &c, 1);
        usleep(3000);  // Espera 3ms entre cada byte
    }

    usleep(100000);  // Espera 100ms antes de intentar leer respuesta

    std::string response = read_response();
    if (!response.empty()) {
        RCLCPP_INFO(this->get_logger(), "Respuesta del PTU: %s", response.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "No se recibió respuesta del PTU.");
    }
}


std::string PTUNode::read_response() {
    char buf[100];
    memset(buf, 0, sizeof(buf));

    int n = read(serial_fd_, buf, sizeof(buf) - 1);  // dejamos 1 byte para el '\0'
    if (n > 0) {
        return std::string(buf, n);
    } else {
        return "";
    }
}

void PTUNode::command_callback(const std_msgs::msg::String::SharedPtr msg) {
    send_command(msg->data);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PTUNode>());
    rclcpp::shutdown();
    return 0;
}
