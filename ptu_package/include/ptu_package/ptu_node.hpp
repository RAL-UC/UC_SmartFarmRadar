// estructura del nodo
#pragma once // inclusion unica 

#include "rclcpp/rclcpp.hpp" // librería base de ROS 2 en C++
#include "std_msgs/msg/string.hpp"
#include <string>
#include <termios.h> // controlar la configuración del puerto serial

// definicion de clase que herede de un nodo ros2
class PTUNode : public rclcpp::Node {
public:
    PTUNode(); // constructor: iniciar node y abrir puerto serie
    ~PTUNode(); // destructor: cerrar puerto serie al destruir el objeto

private:
    int serial_fd_; // direccion del puerto serial
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
    bool open_serial(const std::string &device); // abrir y configurar puerto serie
    void send_command(const std::string &cmd); // envio de comando ascii
    std::string read_response(); // leer respuesta
    void command_callback(const std_msgs::msg::String::SharedPtr msg);
};