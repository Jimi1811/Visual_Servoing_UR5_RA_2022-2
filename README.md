# Visual_Servoing_UR5_RA_2022-2
En este repositorio se presentan los códigos implementados para la aplicación de Visual Servoing en el brazo robótico UR5 para tomar muestras de racimos de uvas.

## Instalación

Al crear un catkin workspace, entra al directorio src.

  ```
  $ roscd; cd ../src
  ```
  
Clonar los siguientes repositorios para tener el modelo del UR5 y el gripper respectivamente. 

  ```
  $ git clone https://github.com/utecrobotics/ur5
  $ git clone https://github.com/utecrobotics/robotiq
  ```
  
Construir usando catkin_make

  ```
  $ cd ..
  $ catkin_make
  ```

