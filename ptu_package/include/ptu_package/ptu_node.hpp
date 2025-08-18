// estructura del nodo
#pragma once // inclusion unica 

#include "rclcpp/rclcpp.hpp" // librería base de ROS 2 en C++
#include "std_msgs/msg/string.hpp" // manejo de mensajes de ros
#include <string> // manipulacion de cadenas 
#include <mutex>
#include <chrono>

// definicion de clase que herede de un nodo ros2
class PTUNode : public rclcpp::Node {
public:
    PTUNode(); // constructor: iniciar node y abrir puerto serie
    ~PTUNode(); // destructor: cerrar puerto serie al destruir el objeto

private:
    //bool initialized_{false}; // estado de inicializacion

    std::string port_; // ruta del dispositivo serial
    int serial_fd_ = -1; // direccion del puerto serial
    bool serial_connected_ = false; // verificacion de conexion

    int retry_count_ = 0;
    int max_retries_ = 1;
    std::chrono::seconds retry_interval_{5};

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_; // variable de suscripcion a un topico de texto en ros
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_; // variable de publicacion de un topico de texto en ros
    rclcpp::TimerBase::SharedPtr reconnect_timer_; // temporizador de reconexion

    bool open_serial(const std::string &device); // abrir y configurar puerto serie
    std::string read_response(std::chrono::milliseconds total_timeout = std::chrono::milliseconds(600)); // leer respuesta
    //void initialize_hardware();
    void try_reconnect();  // reconexion automatica
    void send_command(const std::string &cmd); // envio de comando ascii
    void command_callback(const std_msgs::msg::String::SharedPtr msg); // funcion de llamada para manejar los mensajes de la suscripcion
    void close_serial_nolock();

    std::mutex serial_mtx_;
};