# rp3ba
Robot paralelo de 3 brazos antropomorfos para simulación e implementación con motores dynamixel

Para leer las posiciones y velocidades de los dynamixel a una frecuencia alta es necesario modificar el latency_timer del FTDI de la U2D2 (la cual por lo general esta 16ms)

Crear una regla udev:
```
sudo nano /etc/udev/rules.d/99-ftdi-latency.rules
```
Contenido:
```
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```
Ejecutar:
```
sudo udevadm control --reload-rules
sudo udevadm trigger
```
