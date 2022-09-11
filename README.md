# Paquete RMD
En este paquete se encuentran los archivos URDF, STL, launch y scrips 
para la implementación de un Sistema de Múltiples Agentes. La versión de 
ROS utilizada es noetic.

1.- Debe crearse un espacio de trabajo llamado "SMA" donde se almacene esta carpeta RMD.
2.- Ejecutar el comando "catkin_make" en una terminal desde el espacio de trabajo.
3.- Ejecutar el comando "source devel/setup.bash".
4.- Lanzar a ejecusión el archivo "sma.launch".
5.- Lanzar a ejecusión el controlador que se desee ("controlh.launch" o "controlc.launch").

NOTA: por default la función de referencia es un polinomio Bézier. Cambie las posiciones 
deseadas desde el archivo de control en la carpeta scrips "controlc1" (o "controlh1"
dependiendo del controlador). Tambien esta programada la función leminiscata(). 
