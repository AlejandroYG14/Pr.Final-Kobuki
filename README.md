# Proyecto Final Kobuki

## Sistema de detección y recogida de objetos mediante comandos de voz

Este proyecto implementa un sistema que permite al robot Kobuki escuchar comandos de voz para buscar y recoger objetos específicos (botellas, pelotas, etc.) y transportarlos a diferentes estaciones definidas. El sistema es capaz de diferenciar entre objetos y decidir qué estación es la más adecuada para cada uno.

### Componentes principales

1. **Reconocimiento de voz**: Utiliza audio_common y pocketsphinx para interpretar comandos de voz.
2. **Detección de objetos**: Emplea YOLO para detectar objetos en las imágenes de la cámara.
3. **Sistema de comportamiento**: Utiliza Behavior Trees para coordinar las acciones del robot.
4. **Control del robot**: Maneja la navegación y manipulación de objetos.
5. **Gestión de estaciones**: Administra tres estaciones diferentes donde los objetos pueden ser entregados.

### Estructura de nodos

- **VoiceCommandNode**: Procesa los comandos de voz del usuario.
- **ObjectDetectionNode**: Detecta objetos utilizando YOLO y crea un mapa 3D.
- **BehaviorTreeNode**: Coordina las acciones de alto nivel usando Behavior Trees.
- **KobukiControlNode**: Controla los movimientos del robot Kobuki.
- **StationManager**: Gestiona las estaciones y decide dónde debe ir cada objeto.

### Flujo de ejecución

1. El usuario da un comando de voz (ej. "buscar botella").
2. El sistema activa la detección de objetos con YOLO.
3. Cuando se encuentra el objeto, el robot navega hacia él.
4. El robot recoge el objeto.
5. El sistema decide qué estación es la apropiada para ese objeto.
6. El robot navega a la estación seleccionada.
7. El robot deposita el objeto en la estación.

### Requisitos

- ROS 2 Humble o superior
- Biblioteca YOLO para detección de objetos
- Biblioteca Behavior Tree CPP v3
- Pocketsphinx para reconocimiento de voz
- Robot Kobuki o simulación compatible

### Configuración

El sistema está configurado para trabajar con tres estaciones diferentes, cada una especializada en ciertos tipos de objetos:

- **Estación 1**: Acepta botellas y vasos
- **Estación 2**: Acepta pelotas y juguetes
- **Estación 3**: Acepta libros y cajas

### Uso

```bash
# Lanzar la simulación
ros2 launch kobuki_yolo_bt kobuki_fetch_object.launch.py use_sim:=true

# Lanzar con robot real
ros2 launch kobuki_yolo_bt kobuki_fetch_object.launch.py use_sim:=false
```

### Customización

Se pueden modificar los objetos reconocibles y las estaciones editando los archivos de configuración:

- `config/stations.yaml`: Define las posiciones y objetos permitidos en cada estación.
- Behavior Tree: `config/kobuki_fetch_object_bt.xml`
