# libserial library changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/)

## [2.3] - 2025.03.17

### Changed
- All functions now trigger exceptions when an unexpected error is received from the device handler.
- Functions waitSend, setLine and setBlocking now return void.
- Function getLine now returns bool instead of int.

## [2.2] - 2025.03.08

### Added
- Move constructor.
- Copy constructor (deleted).

### Changed
- La estructura usada en getBaudCode ahora es privada de la función y no de la clase.

## [2.1] - 2025.03.05

### Changed
- Cambiado el tipo del parámetro de constructor devname de const char* a const std::string&
- Uso de steady_clock en lugar de system_clock.
- write() ahora despacha una excepción si hay un error de escritura (salvo escritura no bloqueante).
- nanoconsole renombrado a nanoterm.

### Fixed
- Ahora se hace cleanup en el constructor antes de lanzar una excepción.
- Faltaba el texto de una excepción.

## [2.0] - 2025.03.04

### Added
- nanoconsole: Programa de test que funciona como una consola cutre.

### Changed
- Clase cambiada para soportar el concepto RAII:
  - El puerto se abre en el constructor y se cierra en el destructor.
  - Eliminadas las funciones open() y close().
  - Tratamiento de errores mediante excepciones.
- Los nombres de los métodos ahora siguen el patrón camelCase.
- Tests cambiados para usar los nuevos nombres y excepciones.
- Indentación a 4 espacios.

## [1.8] - 2025.02.28

### Added
- Nueva función libserial::version() para identificar la versión de la biblioteca.

### Changed
- Eliminada la dependencia de libUtility.
- Uso de std::chrono en lugar de libUtility::Timer para medir el tiempo.
- changelog pasa a formato .md

## [1.7] - 2020.05.22

### Added
- Añadida la función WaitSend().

### Fixed
- Corregido un error en PendingWrite: si solo quedaba el byte del shift register, se devolvía 0.

## [1.6.0] - 2019.02.04

### Changed
- Modificación del interfaz de la clase:
  - Los tipos de puntero de Read y Write ahora son punteros a void. Fin de la pesadilla.

### Fixed
- El timeout de lectura no se reiniciaba por la llegada de algún carácter.

## [1.5.0] - 2019.02.01

### Changed
- Modificación del interfaz de la clase:
  - El constructor ahora no admite parámetros.
  - La función 'Init' es renombrada a 'Open', que admite nuevos parámetros para especificar el
    dispositivo y la modalidad de apertura (bloqueante o no bloqueante).
  - La velocidad (baudrate) se especifica como valor entero y se convierte internamente.
  - Ampliada la tabla de conversión de velocidades para incluir las nuevas constantes (hasta 4 Mbps)
  - Nueva función Close para cerrar el dispositivo.
  - Nueva función SetBlocking para definir el modo de bloqueo en lectura.
  - IsValid pasa a llamarse IsOpen. El objeto también es válido cuando el puerto está cerrado.
  - Eliminadas las funciones IsTxFIFOempty (redundante) y GetBaudValue (innecesaria).
  - Las funciones PendingRead y PendingWrite ahora devuelven el número de bytes pendientes.
- Actualización de contenido y estilo de la documentación.
- Limpieza de código muerto o inútil.

## [1.4.0] - 2017.05.09

### Changed
- La función Init recibe ahora parámetros enumerados, en lugar de referir a constantes de termios.h

## [1.3.3] - 2016.06.03

### Added
- Nueva función GetBaudValue, reverso de GetBaudCode.

### Changed
- Ampliación de la funcionalidad del programa de pruebas.
- GetBaudValue y GetBaudCode pasan a ser estáticas.

## [1.3.2] - 2016.06.01

### Changed
- El programa de test ahora admite que el mensaje a enviar sea opcional y el tiempo de espera infinito.

## [1.3.1] - 2016.05.27

### Fixed
- Añadido un programa de test.

## [1.3.0] - 2016.05.03

### Added
- Nuevas funciones PendingRead y PendingWrite para determinar si quedan datos en los buffers de entrada y salida.

## [1.2.3] - 2016.04.27

### Fixed
- Eliminadas un par de trazas que andaban por ahí perdidas.

### Changed
- Actualización de la documentación.

## [1.2.2] - 2016.04.01

### Changed
- La identificación de versión se segrega a un fichero fuente independiente version.cpp

## [1.2.1] - 2016.03.09

### Changed
- Modificaciones por el cambio de interfaz del módulo de gestión de versiones.

## [1.2.0] - 2014.04.23

### Changed
- Reordenación de código, moviendo las cabeceras al directorio raíz del módulo.
- Información para doxygen movida a las cabeceras.

## [1.1.4] - 2008.07.15

### Added
- Se incluye función IsTxFIFOempty para comprobar si la UART serie transmitió todos los datos del buffer.

## [1.1.3] - 2008.02.15

### Fixed
- Corrección al fijar la velocidad, a lo POSIX.

## [1.1.2] - 2008.01.24

### Added
- Añadida la función WriteByte para escribir un solo byte.

## [1.1.1] - 2007.11.26

### Fixed
- La función GetVersion no era estática como debería ser.
- Eliminado el prototipo de la función Init antigua, que no debería estar ahí.

## [1.1.0] - 2007.10.24

### Changed
- Modificada la función Init para admitir parámetros en un formato más "coherente".
- Añadida la función GetBaudCode que convierte valores de velocidad en códigos para Init.

## [1.0.0] - 2006.10.24

### Added
Actualización de la documentación.

### Changed
Cambios estéticos.

## [0.2.1] - 2006.01.30

### Changed
- Reemplazado gettimeofday por TimerClass.

## [0.2.0] - 2006.01.23

### Added
La librería comienza su vida independiente del módulo tci.
