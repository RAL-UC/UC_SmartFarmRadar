// estructura del nodo
#pragma once // inclusion unica 

#include "rclcpp/rclcpp.hpp" // librería base de ROS 2 en C++
#include "std_msgs/msg/string.hpp" // manejo de mensajes de ros
#include <string> // manipulacion de cadenas 
#include <termios.h> // controlar la configuración del puerto serial

// definicion de clase que herede de un nodo ros2
class PTUNode : public rclcpp::Node {
public:
    PTUNode(); // constructor: iniciar node y abrir puerto serie
    ~PTUNode(); // destructor: cerrar puerto serie al destruir el objeto

private:
    int serial_fd_ = -1; // direccion del puerto serial
    bool serial_connected_ = false; // verificacion de conexion
    std::string port_; // ruta del dispositivo serial

    int retry_count_ = 0;
    int max_retries_ = 1;

    bool open_serial(const std::string &device); // abrir y configurar puerto serie
    void send_command(const std::string &cmd); // envio de comando ascii
    std::string read_response(); // leer respuesta
    void command_callback(const std_msgs::msg::String::SharedPtr msg); // funcion de llamada para manejar los mensajes de la suscripcion

    void try_reconnect();  // reconexion automatica

    rclcpp::TimerBase::SharedPtr reconnect_timer_; // temporizador de reconexion
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_; // variable de suscripcion a un topico de texto en ros
};