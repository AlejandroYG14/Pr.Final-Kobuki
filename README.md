# Proyecto final Kobuki - Navegación y detección de personas

Este proyecto implementa un sistema que permite al robot Kobuki navegar usando Nav2 hasta un punto específico, detectar personas usando YOLO, y volver al punto de origen.

### Flujo de Ejecución

1. El robot navega hasta un waypoint predefinido usando Nav2.
2. Al llegar al waypoint, activa el detector YOLO para buscar personas.
3. Cuando detecta una persona, gira y se ajusta para mantenerla en el campo de visión.
4. Después de la detección, el robot navega de vuelta al punto de origen.

### Componentes principales

1. **Navegación**: 
   - Utiliza Nav2 para la navegación autónoma
   - Sistema de waypoints para definir los puntos de destino
   - Planificación de rutas y evitación de obstáculos

2. **Detección**:
   - YOLO para detección de personas en tiempo real
   - Integración con la cámara RGB-D
   - Transformación de coordenadas 2D a 3D

3. **Control del Robot**:
   - Sistema de control PID para seguimiento suave
   - Integración con el sistema de navegación
   - Gestión de transformadas (TF2)

### Estructura del Sistema

- **TFPublisherNode**: Publica las transformadas de las detecciones de YOLO
- **TFSeekerNode**: Controla el robot para mantener el objetivo en vista
- **NavigationNode**: Maneja la navegación autónoma con Nav2
- **BehaviorTreeNodes**: Coordina las diferentes acciones del robot

### Uso

```bash
# Lanzar el sistema completo


# El robot automáticamente:
# 1. Navegará al waypoint (4.08, -7.84)
# 2. Buscará una persona
# 3. Volverá al origen (0, 0)
```

### Customización

Se pueden modificar los siguientes parámetros:
- Coordenadas del waypoint en `src/bt_nav_main.cpp`
- Parámetros de control PID en `src/bt_nav/PIDController.cpp`

### Mapa usado
![map](https://github.com/user-attachments/assets/e44a9900-7fe4-4ade-adb9-a51916fc8e95)

### Vídeo de demostración
https://github.com/user-attachments/assets/e5415a51-c798-4c5d-8ebe-df0e736daf5b

