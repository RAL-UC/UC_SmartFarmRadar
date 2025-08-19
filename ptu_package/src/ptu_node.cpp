#include "ptu_package/ptu_node.hpp" // cabecera con definicion de la clase PTUNode
// acceder al puerto serial y manejar mensajes
#include "std_msgs/msg/string.hpp" // manejar mensajes de ros
#include <unistd.h> // acceso a llamadas al sistema operativo
#include <fcntl.h> // control de archivos en sistemas POSIX
#include <cstring> // manipulacion de cadenas

#include <termios.h>
#include <sstream>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

PTUNode::PTUNode() : Node("PTU_node") {
    port_ = this->declare_parameter("serial_port", "/dev/ttyUSB0"); // asignar puerto serial

    serial_connected_ = open_serial(port_); // realizar conexion
    if (serial_connected_) {
        RCLCPP_INFO(this->get_logger(), "Conectado a PTU en %s", port_.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "No se pudo abrir el puerto: %s. Intentando reconectar...", port_.c_str());
    }
    
     // intentar reconexión automática cada 5 segundos si está desconectado
    reconnect_timer_ = this->create_wall_timer(retry_interval_, std::bind(&PTUNode::try_reconnect, this));

    // suscripcion a un topico
    command_subscriber_ = this->create_subscription<std_msgs::msg::String>("ptu_cmd", 10, std::bind(&PTUNode::command_callback, this, std::placeholders::_1));

    // publisher
    feedback_pub_ = this->create_publisher<std_msgs::msg::String>("ptu_response", 50);

    // no se comprobara inicializacion si no tan solo conexion
    // restriccion: se debe esperar correctamente a que el disposivo encienda y ejecute una breve rutina de inicializacion interna
    //initialize_hardware(); // logica de inicializacion
}
// cerrar descriptor de archivo asociado a puerto serial si esta abierto
PTUNode::~PTUNode() {
  std::lock_guard<std::mutex> lk(serial_mtx_);
  close_serial_nolock();
}

void PTUNode::close_serial_nolock() {
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  serial_connected_ = false;
}

// metodo de clase PTUNode, abrir y configurar el puerto serial indicado, devuelve true si tuvo exito, false si falla
bool PTUNode::open_serial(const std::string &device) {
    std::lock_guard<std::mutex> lk(serial_mtx_);

    if (serial_fd_ >= 0) {
        close_serial_nolock();
    }
    // apertura del puerto en modo lectura y escritura
    // no convierte este puerto en la terminal de control del proceso
    // espera que las escrituras se completen sin buffer
    
    serial_fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
        serial_connected_ = false;
    return false;
    }

    struct termios tty; // estructura para configurar el puerto
    memset(&tty, 0, sizeof tty); // pone todos los bytes de la estructura tty en cero
    if (tcgetattr(serial_fd_, &tty) != 0) {
        close_serial_nolock();
        serial_connected_ = false;
        return false;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag = CS8 | CLOCAL | CREAD; // 8 bits, sin modem control, lectura activa
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // sin paridad, 1 stop bit, sin control de flujo RTS/CTS
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // sin XON/XOFF
    // desactiva todas las transformaciones de caracteres de entrada (saltos de línea, retorno de carro, etc)
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // bytes crudos, sin conversión de entrada
    tty.c_lflag = 0; // sin procesamiento de entrada por línea
    tty.c_oflag = 0; // sin procesamiento de salida
    tty.c_cc[VMIN] = 0; // 1 byte de retorno
    tty.c_cc[VTIME] = 5; // timeout de 0.5s

    tcflush(serial_fd_, TCIOFLUSH); // limpia tanto el buffer de entrada como de salida del puerto
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        close_serial_nolock();
        serial_connected_ = false;
        return false;
    }

    serial_connected_ = true;
    return true;
    //return tcsetattr(serial_fd_, TCSANOW, &tty) == 0; // aplica cambios inmediatos
}

// recibe el comando que se quiere enviar y no retorna nada
void PTUNode::send_command(const std::string &cmd) {
    std::unique_lock<std::mutex> lk(serial_mtx_);
    // verificacion de conexion y si hay un descriptor de archivo valido
    if (!serial_connected_ || serial_fd_ < 0) {
        RCLCPP_WARN(this->get_logger(), "Puerto serial no conectado. Ignorando comando.");
        serial_connected_ = false;
        return;
    }
    std::string full_cmd = cmd + '\r'; // comandos con retorno de carro

    // mostrar bytes que se enviarán (debug en hex)
    //std::ostringstream debug;
    //for (unsigned char c : full_cmd) {
    //    debug << std::hex << std::showbase << (int)c << " ";
    //}

    //RCLCPP_INFO(this->get_logger(),
    //    "Comando recibido: '%s'\nBytes enviados: %s",
    //    cmd.c_str(), debug.str().c_str()
    //);

    // evita confundir respuestas antiguas con las nuevas
    tcflush(serial_fd_, TCIFLUSH); // limpia el buffer de entrada antes de enviar

    // enviar carácter por carácter con pequeña pausa para emular 'screen'
    // verificacion de conexion con cierre de descriptor y marca de desconexion
    //for (char c : full_cmd) {
    //    if (write(serial_fd_, &c, 1) < 0) {
    //        RCLCPP_ERROR(this->get_logger(), "Error al escribir en el puerto. Marcando como desconectado.");
    //        if (serial_fd_ >= 0) {
    //            close(serial_fd_);
    //            serial_fd_ = -1;
    //        }
    //        serial_connected_ = false;
    //        return;
    //    }
    //    usleep(3000); // espera 3ms entre cada byte
    //}

    // enviar comando de una vez
    ssize_t written = write(serial_fd_, full_cmd.data(), full_cmd.size());
    if (written < 0 || static_cast<size_t>(written) != full_cmd.size()) {
        RCLCPP_ERROR(this->get_logger(),
                    "Error al escribir en el puerto (written=%ld). Marcando desconectado.",
                    static_cast<long>(written));
        close_serial_nolock();
        return;
    }

    lk.unlock();
    usleep(100000); // espera 100ms antes de intentar leer respuesta
    lk.lock();

    std::string response = read_response(600ms);
    std_msgs::msg::String msg_out;
    if (!response.empty()) {
        RCLCPP_INFO(this->get_logger(), "Respuesta del PTU: %s", response.c_str());
        msg_out.data = response;
    } else {
        RCLCPP_WARN(this->get_logger(), "No se recibió respuesta del PTU.");
        msg_out.data = "Sin respuesta";
    }
    feedback_pub_->publish(msg_out);
}


std::string PTUNode::read_response(std::chrono::milliseconds total_timeout) {
    std::string out;
    char buf[128]; // buffer de 100 caracteres
    //memset(buf, 0, sizeof(buf)); // llenar de 0 el arreglo
    const auto start = std::chrono::steady_clock::now();

    //int n = read(serial_fd_, buf, sizeof(buf) - 1);  // dejamos 1 byte para el '\0'
    //if (n > 0) { // cantidad de bytes leidos
    //return std::string(buf, n);
    //} else if (n == 0) {
    //    RCLCPP_WARN(this->get_logger(), "No se leyó ningún byte del PTU.");
    //    return "";
    //} else {
    //    RCLCPP_ERROR(this->get_logger(), "Error al leer del puerto. Marcando como desconectado.");
    //    if (serial_fd_ >= 0) {
    //        close(serial_fd_);
    //        serial_fd_ = -1;
    //    }
    //    serial_connected_ = false;
    //    return "";
    //
    while (true) {
        int n = read(serial_fd_, buf, sizeof(buf));
        if (n > 0) {
            out.append(buf, n);
            if (out.find('\n') != std::string::npos || out.find('\r') != std::string::npos) {
                break; // llegó fin de línea
            }
        } else if (n == 0) {
            // nada disponible ahora; ¿expiró el timeout total?
            if (std::chrono::steady_clock::now() - start > total_timeout) break;
            usleep(10000); // 10ms y reintenta
        } else {
            // error de lectura
            RCLCPP_ERROR(this->get_logger(), "Error al leer del puerto. Marcando como desconectado.");
            close_serial_nolock();
            return "";
        }
    }
    return out;
}

void PTUNode::command_callback(const std_msgs::msg::String::SharedPtr msg) {
    send_command(msg->data);
}

void PTUNode::try_reconnect() {
  std::lock_guard<std::mutex> lk(serial_mtx_);

  if (serial_connected_) return;

  if (max_retries_ >= 0 && retry_count_ >= max_retries_) {
    RCLCPP_ERROR(this->get_logger(), "Se alcanzó el límite máximo de reconexiones (%d). Apagando ROS...", max_retries_);
    reconnect_timer_->cancel();
    close_serial_nolock();   // cierra descriptor si quedaba abierto
    rclcpp::shutdown();      // APAGA TODO ROS (todos los nodos)
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Intentando reconectar al PTU en %s...", port_.c_str());
  if (open_serial(port_)) {
    RCLCPP_INFO(this->get_logger(), "Reconexión exitosa al PTU.");
    retry_count_ = 0;
  } else {
    retry_count_++;
  }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<PTUNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::cerr << "Excepción no controlada: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Excepción desconocida atrapada en el nodo." << std::endl;
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return 0;
}
