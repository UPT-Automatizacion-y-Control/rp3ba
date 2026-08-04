# rp3ba
Robot paralelo de 3 brazos antropomorfos para simulación e implementación con motores dynamixel

Clonar el repositorio
```
cd ~/ros2_ws/src/
git clone https://github.com/UPT-Automatizacion-y-Control/rp3ba.git
```
Antes de compilar resolver las dependencias
```
cd ~/ros2_ws/
rosdep install -i --from-path src --rosdistro jazzy -y
```
Compilar el paquete
```
colcon build
```
Dar permisos al puerto
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

Para mover el robot virtual:
```
ros2 launch rp3ba virtual.launch.py 
ros2 topic pub /modo std_msgs/msg/Int16 "{data: 2}" --once
```

Para mover el robot real:
```
ros2 launch rp3ba robot.launch.py 
ros2 topic pub /modo std_msgs/msg/Int16 "{data: 2}" --once
```
