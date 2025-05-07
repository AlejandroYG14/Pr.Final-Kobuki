# Proyecto Final Kobuki

## Sistema de detección y recogida de objetos

Este proyecto implementa un sistema que permite al robot Kobuki navegar por un mapa predefinido usando Nav2, buscar y recoger objetos específicos en diferentes estaciones, y moverse entre ellas. El sistema utiliza interacción por línea de comandos para confirmar la recogida de objetos.

### Componentes principales

1. **Navegación**: Utiliza Nav2 para navegar por el mapa y llegar a las estaciones.
2. **Detección de objetos**: Emplea OpenCV para detectar objetos en las imágenes de la cámara.
3. **Sistema de comportamiento**: Utiliza Behavior Trees para coordinar las acciones del robot.
4. **Control del robot**: Maneja la navegación y manipulación de objetos.
5. **Gestión de estaciones**: Administra las estaciones donde se encuentran los objetos.

### Estructura de nodos

- **ObjectDetectionNode**: Detecta objetos utilizando OpenCV.
- **BehaviorTreeNode**: Coordina las acciones de alto nivel usando Behavior Trees.
- **KobukiControlNode**: Controla los movimientos del robot Kobuki.
- **StationManager**: Gestiona las estaciones y la navegación entre ellas.

### Flujo de ejecución

1. El sistema recibe un comando para buscar un objeto específico.
2. El robot navega a la estación inicial usando Nav2.
3. En la estación, el robot busca el objeto usando OpenCV.
4. Cuando encuentra el objeto, se acerca a él.
5. El sistema espera confirmación del usuario por línea de comandos (presionar 'S' para confirmar).
6. Después de recoger el objeto, el robot navega a la siguiente estación.
7. El proceso se repite hasta que se han visitado todas las estaciones.

### Requisitos

- ROS 2 Humble o superior
- Nav2 para navegación
- OpenCV para detección de objetos
- Biblioteca Behavior Tree CPP v3
- Robot Kobuki o simulación compatible

### Configuración

El sistema está configurado para trabajar con un mapa predefinido y varias estaciones donde se encuentran los objetos. Las posiciones de las estaciones se definen en el archivo de configuración.

### Uso

```bash
# Lanzar la simulación
ros2 launch kobuki_yolo_bt kobuki_fetch_object.launch.py use_sim:=true

# Lanzar con robot real
ros2 launch kobuki_yolo_bt kobuki_fetch_object.launch.py use_sim:=false

# Enviar comando para buscar un objeto
ros2 topic pub /object_request std_msgs/String "data: 'botella'"

# Durante la ejecución, cuando el robot esté cerca del objeto:
# Presionar 'S' para confirmar que el objeto ha sido recogido
```

### Customización

Se pueden modificar las estaciones y sus posiciones editando los archivos de configuración:

- `config/stations.yaml`: Define las posiciones de las estaciones en el mapa.
- Behavior Tree: `config/kobuki_fetch_object_bt.xml`
