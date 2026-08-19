# rp3ba
Robot paralelo de 3 brazos antropomorfos para simulación e implementación con motores dynamixel

## Instalación

Clonar el repositorio:
```
cd ~/ros2_ws/src/
git clone https://github.com/UPT-Automatizacion-y-Control/rp3ba.git
```
Antes de compilar resolver las dependencias:
```
cd ~/ros2_ws/
rosdep install -i --from-path src --rosdistro jazzy -y
```
Compilar el paquete:
```
cd ~/ros2_ws/
colcon build
```
Dar permisos al puerto:
```
sudo usermod -a -G dialout $USER
```
Para leer las posiciones y velocidades de los dynamixel a una frecuencia alta es necesario modificar el latency_timer del FTDI de la U2D2 (la cual por lo general esta 16ms)

Crear una regla udev:
```
sudo nano /etc/udev/rules.d/99-ftdi-latency.rules
```
Escribir este contenido:
```
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```
Guardar, cerrar y ejecutar:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Reiniciar la computadora

## Uso básico

Para cargar los nodos del robot se usa:
```
ros2 launch rp3ba rp3ba.launch.py
```

Para publicar una referencia se usa:
```
ros2 topic pub /modo std_msgs/msg/Int16 "{data: 1}" --once
```
El topico /modo define la trayectoria de referencia:

0. Home
1. Traslación X
2. Traslación Y
3. Traslación Z
4. Rotacion X
5. Rotacion Y
6. Rotacion Z
7. Traslacion combinada XY
8. Rotación combinada XY
9. Definida por el usuario en el topico /user_goal

## Opciones avanzadas

El archivo rp3ba.launch.py tiene dos argumentos:

- operation_mode
- PID

El argumento operation_mode define si se usa el robot real o virtual:
```
# (default) De momento solo simula la cinemática
ros2 launch rp3ba rp3ba.launch.py operation_mode:=virtual
```
```
# Para conectarse al robot real usando U2D2 o equivalente
ros2 launch rp3ba rp3ba.launch.py operation_mode:=real
```

El argumento PID define la sintonización del PID de los motores dynamixel (se ignoran en operation_mode:=virtual):
```
# (default) Control P de ganancias bajas, para usarse como rehabilitador
ros2 launch rp3ba rp3ba.launch.py operation_mode:=real PID:=soft
```
```
# Control PD de ganancias altas, para usarse en seguimiento de trayectorias
ros2 launch rp3ba rp3ba.launch.py operation_mode:=real PID:=accuracy
```
