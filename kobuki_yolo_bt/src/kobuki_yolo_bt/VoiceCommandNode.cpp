#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "kobuki_yolo_bt/VoiceCommandNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "audio_common_msgs/msg/audio_data.hpp"

namespace kobuki_yolo_bt
{

using std::placeholders::_1;

VoiceCommandNode::VoiceCommandNode()
: Node("voice_command_node")
{
  // Inicializar lista de objetos reconocibles
  recognized_objects_ = {
    "botella", "pelota", "vaso", "juguete", "libro", "caja"
  };
  
  // Inicializar lista de comandos reconocibles
  recognized_commands_ = {
    "buscar", "ir", "recoger", "traer"
  };
  
  // Crear suscripción a datos de audio
  audio_sub_ = create_subscription<audio_common_msgs::msg::AudioData>(
    "audio_data", 10, std::bind(&VoiceCommandNode::audio_callback, this, _1));
  
  // Crear publicador para comandos reconocidos
  command_pub_ = create_publisher<std_msgs::msg::String>("voice_command", 10);
  
  RCLCPP_INFO(get_logger(), "Nodo de comandos de voz inicializado");
  RCLCPP_INFO(get_logger(), "Esperando comandos de voz...");
}

void VoiceCommandNode::audio_callback(const audio_common_msgs::msg::AudioData::ConstSharedPtr & msg)
{
  // En un sistema real, aquí se procesaría el audio usando una biblioteca como PocketSphinx
  // para convertir el audio en texto y luego extraer comandos.
  
  // En esta implementación simulada, simplemente imprimimos que se recibió audio
  RCLCPP_DEBUG(get_logger(), "Audio recibido: %zu bytes", msg->data.size());
  
  // Simulamos un comando de voz para probar el sistema
  // En un sistema real esto vendría del reconocimiento de voz
  if (msg->data.size() > 1000) {  // Umbral arbitrario solo para simular
    process_command("buscar botella");
  }
}

void VoiceCommandNode::process_command(const std::string & command)
{
  RCLCPP_INFO(get_logger(), "Procesando comando: %s", command.c_str());
  
  // Buscar objetos conocidos en el comando
  std::string detected_object;
  for (const auto & obj : recognized_objects_) {
    if (command.find(obj) != std::string::npos) {
      detected_object = obj;
      break;
    }
  }
  
  // Buscar comandos conocidos
  std::string detected_command;
  for (const auto & cmd : recognized_commands_) {
    if (command.find(cmd) != std::string::npos) {
      detected_command = cmd;
      break;
    }
  }
  
  // Si se reconoció un objeto y un comando, publicar el objeto
  if (!detected_object.empty() && !detected_command.empty()) {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = detected_object;
    command_pub_->publish(std::move(msg));
    
    RCLCPP_INFO(get_logger(), "Comando reconocido: %s %s", 
      detected_command.c_str(), detected_object.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "No se reconoció un comando válido");
  }
}

}  // namespace kobuki_yolo_bt 