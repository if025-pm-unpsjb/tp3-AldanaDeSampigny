[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/VY9he9aE)
# HelloWorld para la placa LM3S6965

Programa de ejemplo que imprime el mensaje `Hello World!`, basado en la [demo de FreeRTOS para QEMU](https://www.freertos.org/cortex-m3-qemu-lm3S6965-demo.html).

## Docker

Generar la imagen docker con:
```bash
$ make docker-build
```

Esto genera, si no existe ya, una imagen con el nombre `rtsg`.

## Línea de comando

Desde una terminal, dentro del directorio del proyecto, generar una copia con nombre `Makefile.mine` del archivo `Makefile.config`:
```bash
$ cp Makefile.config Makefile.mine
```

Luego, se puede compilar el proyecto mediante el siguiente comando:
```bash
$ ./docker.make
```

Para ejecutar el programa en una máquina virtual QEMU, ejecutar:
```bash
$ ./docker.make qemu
```

Se puede terminar la ejecución presionando `C^A X`. También es posible visualizar la salida gráfica de QEMU mediante VNC conectandose con un cliente a `:0` o accediendo a [`localhost:6900/vnc.html`](http://localhost:6900/vnc.html).

## Importar y compilar en Eclipse

Para importar el proyecto en Eclipse:

1. Seleccionar **[File > Import]**.
2. En la nueva ventana:
    - Seleccionar **[C/C++ > Existing Code as Makefile Project]** y hacer clic en **[Next >]**.
    - En **[Existing Code Location]** indicar el *path* en donde se descargó o clonó el repositorio (usar el botón **[Browse...]**).
    - En **[Toolchain for Indexer]** seleccionar la opción *ARM Cross GCC* (¡importante!).

Luego, para configurar el proceso de compilación hay dos alternativas:

### Si se utiliza Docker:
1. Seleccionar **[Project > Properties]**
2. En la nueva ventana:
    - Seleccionar **[C/C++ Build > Settings]** en la lista de la izquierda.
    - En la sección derecha, hacer clic en la pestaña **[Container settings]**
    - Tildar la opción **[Build inside Docker image]**
    - En **Connection** seleccionar la conexión al *daemon* Docker (generalmente `unix:///var/run/docker.sock`)
    - En **Image** seleccionar `rtsg:latest`
    - En **Data volumes** mapear el path `/app` con el path del proyecto.
    - Finalmente, cliquear **[Apply and close]**

### Si se utilizo la guía de instalación paso a paso:
El proyecto debe aparecer ahora en la vista *Project Explorer*:
   - Hacer clic derecho sobre el mismo y seleccionar **[Properties]** en el menú contextual.
   - En la nueva ventana, en la sección izquierda, seleccionar **[C/C++ Build > Settings]**. En la sección derecha de la ventana, hacer clic en la pestaña **[Toolchains]**. Verificar que el campo *Name* indique *GNU MCU Eclipse ARM Embedded GCC (arm-none-eabi-gcc)* o similar.
   - Hacer clic en **[Apply and Close]**.

Finalmente, para compilar el proyecto se puede:

- Hacer clic derecho sobre el proyecto en la vista *Project Explorer* y seleccionar **[Build]** en el menú contextual.
- Seleccionar en la barra de menúes de Eclipse **[Project > Build Project]**.
- Hacer clic en el ícono *Build* (un martillo).

Si el proyecto compilo correctamente, en la vista **[Console]** debe indicarse que se generó correctamente el archivo `build/main.elf`.

## Debugging desde Eclipse con QEMU

Primero configurar el perfil de _debugging_:

1. Seleccionar **[Run > Debug Configurations...]** en el menú de Eclipse.
2. En la nueva ventana, hacer doble clic sobre **[GDB Hardware Debugging]** en el menú izquierdo. Esto crea una nueva configuración basada en el perfil, con el nombre del proyecto activo.
3. Seleccionar el nuevo perfil creado (*lm3s6965evb-helloworld-makefile*).
4. En el panel derecho:
    - En la pestaña **[Main]**:
      - En el *Project* debe indicar el nombre del proyecto (*lm3s6965evb-helloworld-makefile*).
      - En el campo *C/C++ Application* se debe indicar `build/main.elf`.
      - Seleccionar **Use workspace settings**.
    - En la pestaña **[Debugger]**:
      - El campo *GDB Command* debe decir `arm-none-eabi-gdb`.
      - Seleccionar **Use remote target**.
      - En *GDB Connection String* completar `localhost:1234`.
    - En la pestaña **[Startup]**:
      - Desmarcar **Reset and delay**, **Halt** y **Load image** si estuvieran seleccionados.
      - Seleccionar la opción **Load symbols** y **Use project binary: main.elf**.
      - Seleccionar la opción **Set breakpoint at:** y en el campo adyacente indicar `main`.
      - Seleccionar la opción **Resume**.
    - En la pestaña **[Common]**, seleccionar la opción **[Shared file:]**, indicando en el campo el nombre del proyecto. De esta manera la configuración para debugging es guardada en un archivo `*.launch` dentro del proyecto.
    - Hacer clic en el botón **[Apply]**, no cerrar la ventana aún.

Luego, para ejecutar QEMU, tenemos dos opciones:

### Ejecutar QEMU instalado o descargado localmente

Si se instalo QEMU localmente (por ejemplo en el directorio `~/setr/qemu`, ejecutar el siguiente comando desde una terminal:

```
~/setr/qemu/bin/qemu-system-arm -kernel ./build/main.elf -S -s -machine lm3s6965evb
```

Luego, en Eclipse hacemos clic en el botón **[Debug]**, y cuando Eclipse nos pregunte si queremos cambiar a la perspectiva de _Debugging_ le decimos que sí (_switch_).

Si todo funcionó correctamente, se alcanza el _breakpoint_ en la función `main()` y la ejecución queda detenida en ese punto. Seleccionar **[Run > Resume]** (o presionar **F8**).

### Ejecutar QEMU en un contenedor Docker desde la línea de comando

Si se tiene la imagen Docker, abrir una terminal y ejecutar el siguiente comando:

```bash
$ ./docker.make gdb
```

Luego, en Eclipse hacemos clic en el botón **[Debug]**, y cuando Eclipse nos pregunte si queremos cambiar a la perspectiva de _Debugging_ le decimos que sí (_switch_).

Si todo funcionó correctamente, se alcanza el _breakpoint_ en la función `main()` y la ejecución queda detenida en ese punto. Seleccionar **[Run > Resume]** (o presionar **F8**).

Se puede configurar una acción en Eclipse para ejecutar este *script* sin necesidad de ir a una terminal:

1. Ir al menú **[Run > External Tools > External Tools configurations]**.
2. En la nueva ventana, crear una nueva configuración haciendo doble clic en **Program**.
3. En el campo **[Name]** ingresar `docker.qemu-gdb` o cualquier nombre descriptivo que se quiera.
4. En el campo **[Location]** ingresar el path completo al script `docker.make` del proyecto.
5. En el campo **[Working directory]** ingresar el path completo al directorio del proyecto.
6. Cliquear **[Apply]** para guardar los cambios.

Luego, ejecutando esta acción, se inicia el servidor gdb de QEMU desde Eclipse.

