# stonefish_net

Red de pesca deformable dentro del simulador [Stonefish](https://github.com/patrykcieslak/stonefish), pensada para el tanque CIRTESU con el BlueROV2 Heavy.

La red es una malla de rombo de hilo fino con un marco de cabo, plomos en el borde inferior y boyas en el superior, fondeada al suelo por las cuatro esquinas. La empuja la corriente del escenario (arrastre de Morison), tiene agujeros aleatorios reproducibles y los robots y los sólidos chocan con ella y notan la reacción. Se calcula en la **GPU** (por defecto) o como cuerpo blando de Bullet (ver [Solver en GPU](#solver-en-gpu)).

No modifica Stonefish ni `stonefish_ros2`. El paquete trae su propio ejecutable, `stonefish_net_simulator`. Es el mismo simulador que `stonefish_simulator` (mismas librerías, mismos topics), pero después de leer el escenario `.scn` añade la red, las boyas, los bloques de anclaje y sus fuerzas.

## Requisitos

- ROS 2 **Humble**.
- **Stonefish 1.6** instalado (`/usr/local`), con `stonefish_ros2` de la versión correspondiente en el mismo workspace.
- `bluerov_stonefish` en el workspace: aporta el escenario del tanque, las mallas y los nodos auxiliares (DVL, batería y fugas).
- Eigen 3 y yaml-cpp (`rosdep install --from-paths src --ignore-src -y`).
- Una GPU con OpenGL 4.3 o superior.

## Compilar

Desde la **raíz** del workspace y **en Release**:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select stonefish_net --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Sin `-DCMAKE_BUILD_TYPE=Release` (o `RelWithDebInfo`) el código se compila sin optimizar y la física de la red va varias veces más lenta.

## Lanzar

```bash
ros2 launch stonefish_net net_cirtesu.launch.py
```

En un portátil con gráfica híbrida NVIDIA, lánzalo en la NVIDIA:

```bash
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch stonefish_net net_cirtesu.launch.py
```

Se abre el tanque CIRTESU con el BlueROV y la red a 5 m del origen, colgada de sus boyas e inclinada por la corriente de 0,2 m/s del escenario. El launch arranca también `dvl_to_twist_node`, `battery_status_simulated` y `leak_sensors_simulated` de `bluerov_stonefish`.

### Argumentos

| Argumento                       | Por defecto                         | Para qué                                                                        |
| ------------------------------- | ----------------------------------- | ------------------------------------------------------------------------------- |
| `scenario`                      | `scenarios/bluerov_cirtesu_net.scn` | Escenario `.scn` que se carga (ver [Usar otro escenario](#usar-otro-escenario)) |
| `simulation_data`               | `bluerov_stonefish/data`            | Carpeta de las mallas y texturas del escenario                                  |
| `net_config`                    | `config/net.yaml`                   | Configuración de la red                                                         |
| `simulation_rate`               | `100.0`                             | Pasos de física por segundo.                                                    |
| `damage_seed`                   | `-1`                                | Semilla de los agujeros; `-1` usa la del YAML                                   |
| `rendering_quality`             | `noshadows`                         | `low`, `medium`, `high` o `noshadows` (los hilos parpadean con las sombras)     |
| `window_width`, `window_height` | `1920`, `1080`                      | Tamaño de la ventana                                                            |

Por ejemplo, otra red rota y calidad baja:

```bash
ros2 launch stonefish_net net_cirtesu.launch.py damage_seed:=7 rendering_quality:=low
```

### Qué debe salir en la terminal

Al arrancar, después de cargar el escenario:

```
Red: sin colisión con la malla cóncava 'Tank'
Red: contacto propio con 'bluerov2/Vehicle'
[stonefish_net] red en la GPU: NVIDIA Corporation / NVIDIA GeForce RTX 4060 … / 4.3.0 …, 5157 nudos, 10016 tramos en 6 colores, 20 subpasos por tick
Red: 5157 nudos, 10016 tramos, 4 boyas, 13 plomos
Daño (semilla 1): 3 rotos [3, 14, 6 rombos], 47 hilos sueltos cortados
```

Los números dependen de `net.yaml`. La línea de la GPU sale en el primer tick; si no dice NVIDIA en un portátil con gráfica híbrida, faltan las variables de PRIME del lanzamiento.

## Configuración (`config/net.yaml`)

Unidades del SI y ejes NED del mundo de Stonefish (**z hacia abajo**). Todas las secciones son obligatorias. Si falta un campo o un valor está fuera de rango, el simulador no abre la ventana y dice cuál es (por ejemplo, `mooring.floor_z: …`).

| Sección | Qué controla |
|---|---|
| `net` | Tamaño (`width` × `height`), lado del rombo (`bar`), ángulo de apertura, diámetro, densidad y módulo de Young del hilo (`twine_modulus`) y diámetro del nudo (`knot_diameter`) |
| `pose` | Posición: `x`, centro en `y` y profundidad de la relinga superior (`z_top`) |
| `frame_rope` | Diámetro, densidad y módulo de Young (`modulus`) del cabo del marco, los orinques y el fondeo |
| `leads` | Separación y masa de los plomos del borde inferior |
| `buoys` | Si hay boyas (`enabled`, opcional, `true` por defecto), su separación, el largo del orinque y su masa. Con `enabled: false` no hay ni boyas ni orinques, y los demás campos no hacen falta. La red solo queda sujeta por el fondeo y por el empuje del cabo del marco |
| `mooring` | Profundidad del suelo (`floor_z`) y, para las esquinas inferiores (`bottom`) y las superiores (`top`), a dónde va su línea (`to`) y su holgura (`slack`). Con `to: floor` (por defecto) va a un ancla en el suelo, desplazada `upstream` aguas arriba y `outward` hacia fuera. Con `to: wall` se ata a la pared lateral del tanque, en `y = ±wall_y` y a la altura `wall_z`, frente a su esquina y sin bloque. Con `to: none` no hay línea. La red del tanque CIRTESU cuelga de las dos paredes por sus esquinas superiores, sin fondeo abajo |
| `hydro` | Coeficientes de arrastre normal y tangencial, masa añadida, densidad del agua, rejilla de la corriente (`current_grid`, GPU) y viscosidad para Cn(Re) (`viscosity`) |
| `thruster_jets` | El chorro de los propulsores de los robots sobre la red (`enabled`, `spread`) |
| `solver` | `backend` (`gpu` o `bullet`); subpasos y amortiguamiento interno de los tramos (`damping_time`, GPU); iteraciones, dureza y margen del contacto (Bullet); amortiguamiento global y rozamiento con el suelo |
| `damage` | Agujeros: semilla, número de rotos, su tamaño y fracción de hilos sueltos cortados |
| `robot_contact` | Enlaces de robot con su caja de contacto medida a mano |
| `robot_contact_auto` | Contacto automático para el resto de enlaces de robot |
| `solid_contact_auto` | Contacto automático para los sólidos dinámicos que no son robots (boyas, objetos sueltos) |
| `sleep` | Baldosas dormidas: tamaño del cuadrado, radios para despertar y dormir, velocidad y tiempo de reposo, y cuánto se mueve el borde para despertar (ver [Baldosas dormidas](#baldosas-dormidas)) |
| `diagnostics` | Resumen por segundo en stderr (velocidades, nudos en el suelo, boyas, tiempos y baldosas despiertas) |

`config/spike.yaml` es una red pequeña (1 × 1 m, malla de 5 cm, sin agujeros) para pruebas rápidas:

```bash
ros2 launch stonefish_net net_cirtesu.launch.py net_config:=$(ros2 pkg prefix stonefish_net)/share/stonefish_net/config/spike.yaml
```

### Contacto con los robots y los sólidos

El contacto nativo de Bullet entre un cuerpo blando y un rígido solo mira los nudos, y con una malla de pocos centímetros un robot pasaría entre ellos. Por eso cada enlace de robot, y cada sólido dinámico, toca la red con un contacto propio, aproximado por una **caja redondeada**:

- **`robot_contact`**: cajas medidas a mano (`half_extents`, `rounding`, `center_offset`, en el marco del enlace). La del BlueROV2 Heavy viene puesta. Si un robot o enlace de la lista no existe en el escenario, sale un aviso y se ignora.
- **`robot_contact_auto`**: con `enabled: true`, todos los demás enlaces de todos los robots del escenario reciben una caja automática, la envolvente de su forma de colisión. Cada una sale en la terminal (`Red: contacto automático con '…'`) con sus medidas. La envolvente siempre es igual o más grande que la pieza: si un enlace tiene una forma muy irregular, mídelo a mano en `robot_contact`.
- **`solid_contact_auto`**: lo mismo para los sólidos dinámicos del escenario que no son robots, incluidas las boyas de la red (`Red: contacto automático (sólido) con '…'`). El orinque de cada boya no choca con su propia boya, porque su anclaje lo lleva al ojal. Con `enabled: false`, esos sólidos vuelven al contacto nativo de Bullet (solo los nudos y sin rozamiento).

> **Nombre del BlueROV.** En las versiones recientes de `bluerov_stonefish` el robot se llama `bluerov`, no `bluerov2`. En ese caso verás el aviso `no existe el enlace 'Vehicle' del robot 'bluerov2'` y el ROV usará la caja automática. Para usar la caja medida, cambia `robot: bluerov2` por `robot: bluerov` en tu `net.yaml`.

### Agarre con la pinza

Con `solver.backend: gpu`, una pinza del robot puede **agarrar la red y tirar de ella**. Se configura solo con los nombres de los enlaces que hacen de dedos, sin medidas de la pinza, así que sirve para cualquier robot:

```yaml
grippers:
  - {robot: bluerov2, fingers: [gripper/finger1, gripper/finger2], grip_margin: 0.004, release_gap: 0.005}
```

- Hacen falta **al menos dos enlaces-dedo** con contacto propio (su caja de `robot_contact` o de `robot_contact_auto`). Una pinza de una sola malla no puede agarrar, porque no se sabe cuándo está cerrada.
- **Agarra:** un nudo (o el punto medio de un tramo) que queda a menos de `grip_margin` m de las cajas de dos dedos a la vez se ata al primero de ellos, en el orden de `fingers`, por el punto en que estaba. Desde ese momento lo sigue sin resbalar y el dedo nota el tirón de la red en el tick siguiente.
- **Suelta:** cuando la distancia entre los centros de las cajas de esos dos dedos crece más de `release_gap` m desde que agarró (los centros de las cajas, no los orígenes de los enlaces, que en una pinza de dedos que giran están en la articulación y no se separan). Mientras lo tiene, y hasta que se separan después de soltarlo, los dedos de esa pinza no chocan con ese nudo ni con sus tramos, y esa pinza no lo vuelve a agarrar mientras siga pellizcado.
- En la terminal sale una línea por cada agarre y cada suelta: `[stonefish_net] pinza bluerov2/gripper/finger1+gripper/finger2: agarra 3 nudos (total 3)` y `… suelta 3 nudos (total 0)`.
- Avisos al arrancar: si un dedo no existe o no tiene contacto propio, la pinza se ignora. Si `solver.backend` es `bullet`, `grippers` se ignora entero. Solo los 32 primeros cuerpos de contacto (índices 0 a 31) pueden ignorar nudos: un dedo más allá agarra pero choca con lo que sujeta, y una boya más allá choca con su orinque; las dos cosas salen avisadas.
- **Resbala** (opcional, `max_pull` en N): si en un tick lo agarrado tira de los dedos más que eso, el hilo resbala entre ellos y la pinza lo suelta todo, como al abrirse (y no lo vuelve a agarrar mientras siga pellizcado). Así la fuerza que nota el robot se satura como con una pinza real. Sale `… resbala con 12.3 N (max_pull 10.0 N), suelta 3 nudos`. Sin `max_pull` (o con 0), no resbala nunca.
- Sin `grippers` en el YAML (o con `grippers: []`), todo funciona como antes.

La pinza de `bluerov_stonefish/scenarios/bluerov.scn` sigue comentada y es una sola malla: para usarla hace falta una pinza de dos o más enlaces. Queda para una fase posterior (con las bridas): que una brida cerrada abrace los hilos sin expulsarlos (contacto por pieza de un cuerpo compuesto), que una brida de pocos gramos no oscile con la reacción en el tick siguiente, y el contacto con cables de Stonefish.

### Física del hilo, los nudos y el agua

Para que la fuerza que nota el robot salga de la física y no del solver (spec: `docs/superpowers/specs/2026-10-27-fuerza-realismo-design.md`). Nada de esto añade lanzamientos de shader: son cuentas dentro de los que ya había.

- **Hilo y cabos elásticos** (`net.twine_modulus`, `frame_rope.modulus`, Pa): cada tramo tiene su flexibilidad L / (E · π d²/4), la del hilo o la del cabo. Con `.inf`, inextensible (como antes). Los valores de `config/net.yaml` (1 GPa el hilo de poliamida, 0,5 GPa el cabo) son estimaciones: con un dato medido de la red real, cámbialos. Con estos módulos el error del solver (que estira algo los tramos más cargados) sigue siendo del mismo orden que el estiramiento físico. Solo con `gpu`: Bullet usa su propia rigidez.
- **Amortiguamiento interno** (`solver.damping_time`, s): cada tramo se amortigua como un Kelvin–Voigt (β = damping_time · rigidez), solo a tracción: frena las vibraciones del hilo sin hacer fuerza neta, así que no llega al robot. El global `solver.damping` frena cada nudo respecto al suelo y sí la hace (a 5 cm/s es del orden de un cuarto del arrastre): `config/net.yaml` lo deja en 0, y la red se asienta igual (test `Xpbd.SettlesWithoutGlobalDamping`). Solo con `gpu`.
- **Nudos** (`net.knot_diameter`, m): cada nudo de hilo es una esfera de ese diámetro con su masa y su volumen (con la densidad del hilo) y su arrastre ½ ρ Cd (π/4) d² |u| u.
- **Cn(Re)** (`hydro.viscosity`, m²/s): con ν > 0, el coeficiente normal del hilo es el de un cilindro según su número de Reynolds, Cn = 1 + 10 · Re^(−2/3) (White), y el del nudo el de una esfera, Cd = 24/Re + 6/(1 + √Re) + 0,4: a velocidades bajas el hilo frena más que con `hydro.cn` constante. Con 0, `hydro.cn` y Cd = 0,47.
- **Chorro de las hélices** (`thruster_jets`): el agua que echa cada propulsor de los robots empuja la red, aunque no la toque. Con su empuje, su marco y el diámetro de su hélice (teoría de cantidad de movimiento: la estela sale a √(2|T|/(ρA)) por un chorro contraído de D/(2√2) de radio), un chorro redondo turbulento con un núcleo de 6,2 diámetros y que luego decae como 1/x, con perfil gaussiano de anchura `spread` · x (0,1 por defecto) y hasta 50 diámetros. El agua va contra el empuje. Se suma a la corriente, también con `bullet`. Hasta 16 propulsores (sale `Red: chorro de N propulsores sobre la red`).

### Fuerza de la red sobre los robots

En cada tick el simulador publica la fuerza que la red hace sobre cada robot y cada pinza, como `geometry_msgs/WrenchStamped`, con los dos backends:

| Topic | Qué suma | Marco (`frame_id`) |
|---|---|---|
| `/stonefish_net/wrench/<robot>` | todos los enlaces del robot con contacto propio, pinza incluida | enlace base del robot (`<robot>/base_link`) |
| `/stonefish_net/wrench/<robot>/gripper_<k>` | los dedos de la pinza `k` de `grippers` (en el orden del YAML; solo con `gpu`) | su primer dedo, al que se atan los nudos agarrados (`<robot>/<dedo>`) |

- Es lo que mediría un sensor de fuerza montado en ese marco: la fuerza en sus ejes y el par respecto a su origen (el marco O de Stonefish, no el centro de gravedad).
- Es la reacción que la red acaba de calcular (`SolidContacts::pending`) y que el robot recibe durante el tick siguiente: la del contacto con las cajas y, con pinzas, el tirón de los nudos agarrados. No incluye nada que no pase por la red (el agua sobre el robot, sus propulsores).
- El sello es el reloj del nodo, el mismo que usan los sensores de `stonefish_ros2`, así que cuadra con las imágenes de las cámaras.
- Se publica siempre, aunque nadie escuche, para poder grabarlo con `ros2 bag`. Al arrancar sale una línea `Red: fuerza sobre … cuerpos en …` por topic.

El robot del escenario del tanque se llama `bluerov2` (ver la nota de [Contacto con los robots y los sólidos](#contacto-con-los-robots-y-los-sólidos)):

```bash
ros2 topic echo /stonefish_net/wrench/bluerov2
```

### Baldosas dormidas

Para no calcular la red entera en cada paso, `sleep` la parte en cuadrados de `tile_size` m (por su posición inicial) y congela los que no hacen falta:

- Un cuadrado **se duerme** cuando lleva `settle_time` s con todos sus nudos por debajo de `sleep_speed` m/s y a más de `sleep_radius` m de cualquier robot. Sus nudos se quedan fijos donde estaban y no se calculan ni el agua, ni el arrastre, ni el peso, ni el contacto.
- **Se despierta** cuando un robot llega a menos de `wake_radius` m, o cuando un nudo vecino de su borde se mueve más de `wake_shift` m (la parte despierta tira de él o lo empuja).
- Los orinques, el fondeo y las esquinas ancladas no se duermen nunca.

Al arrancar todo está despierto: la red se asienta primero. Con `diagnostics: {enabled: true}` sale cada segundo cuántas baldosas y nudos se están calculando. Para comparar, pon `sleep.enabled: false`.

Qué hay que saber:
- Un cuadrado dormido no reacciona si cambia la corriente.
- Mientras su vecino no despierta, el borde congelado es rígido: un tirón fuerte del robot se nota algo más duro hasta que el borde se mueve `wake_shift`.
- Bullet sigue pasando por todos los nudos en sus bucles internos (con masa 0 cuestan poco): lo que se ahorra es sobre todo la parte «fuerzas» y «contacto con los robots» del diagnóstico.

### Solver en GPU

Con `solver.backend: gpu` (lo que viene en los YAML), la red se calcula con compute shaders de OpenGL 4.3 en la misma GPU que dibuja Stonefish. No necesita nada que no pida ya Stonefish: ni CUDA ni librerías nuevas. Así se puede bajar `net.bar` mucho más que con Bullet.

- Cada tick de Stonefish se parte en `solver.substeps` subpasos de XPBD, con una pasada por los tramos en cada uno. Más subpasos = hilos menos elásticos: con 20 a 200 Hz (el `simulation_rate` del launch; subpasos de 0,25 ms), el hilo más cargado de `net.yaml` se estira en torno a un 1 %. Cada subpaso son unos 17 lanzamientos de shader seguidos: el coste va casi en proporción a los subpasos por segundo, no al tamaño de la red.
- La flexibilidad de cada tramo sale de su módulo de Young (`net.twine_modulus`, `frame_rope.modulus`) y su amortiguamiento interno de `solver.damping_time` (ver [Física del hilo, los nudos y el agua](#física-del-hilo-los-nudos-y-el-agua)).
- **El hilo solo tira, no empuja**, como uno de verdad. Con Bullet los tramos también resistían la compresión.
- La corriente del océano se muestrea cada tick en una rejilla de `hydro.current_grid` m alrededor de la red.
- Los robots, las boyas y los sólidos sueltos chocan con contacto propio (cajas redondeadas) y notan la reacción en el tick siguiente.
- Cada orinque se une al ojal de su boya teniendo en cuenta la masa y la inercia de la boya, y al acabar el tick la boya queda donde la dejó la red y con la velocidad de lo que se movió. Si recibiera la reacción en el tick siguiente, como los sólidos, la red que cuelga (mucho más pesada que una boya de 0,5 kg) la haría oscilar cada vez más hasta dar NaN.
- `sleep` no se usa: toda la red se calcula en cada tick.

Con `diagnostics: {enabled: true}` sale cada segundo el tiempo de GPU, el que tarda la CPU en mandar los subpasos y el de espera y lectura por tick. Referencia sin Stonefish (`gpu_check bench`, RTX 4060 Laptop, 20 subpasos a 200 Hz, una caja empujando la red): `net.yaml` (bar 7 cm, 5157 nudos) 1,7 ms por tick de los 5 ms que dura. Dentro del simulador cuesta más, porque comparte la GPU con el dibujo: en una prueba con 40 subpasos, la GPU pasó de 4,4 a 2,5 ms por tick al cerrar la ventana.

### Tiempo real

Stonefish pide a Bullet en cada vuelta todo el tiempo de reloj que ha pasado desde la anterior, sin límite de ticks, y el dibujo solo se actualiza entre vueltas. Si un tick cuesta más de lo que simula (5 ms a 200 Hz), cada vuelta tiene que recuperar más ticks que la anterior: el ROV y la red se quedan quietos cada vez más rato y luego saltan, y al cerrar sigue calculando los segundos que debía. El agua no salta porque sus olas se animan al dibujar.

Por eso el simulador de la red lee el reloj con la recuperación acotada a 3 ticks por vuelta (`core::CatchUpClock`): si la física no llega, va a cámara lenta pero fluida, y avisa cada 10 s (`AVISO: la física no llega a tiempo real`). Con `diagnostics` sale además cada segundo una línea `tiempo real: factor …` con el coste medio del tick, lo que se lleva la red y lo que se lleva el resto (Bullet, sensores, Stonefish y ROS). Con un factor por debajo de 1:

- Si la red se lleva la mayor parte, baja `solver.substeps` (los hilos se estiran más) o sube `net.bar`.
- Si se lo lleva el resto, la red no es la causa: baja `simulation_rate` en el launch, la resolución o la frecuencia de las cámaras.

Para comparar con el cuerpo blando de Bullet, pon `solver.backend: bullet` (allí cuentan `iterations` y `contact_hardness`, y `sleep` si está activo). Bullet necesita ticks cortos: a `simulation_rate` 100 la red explota en el primer segundo (con la red de `test/net_test.yaml`) y Bullet se cae en `btSparseSdf::Evaluate`; a 200 va bien.

**Pruebas.** Lo que no necesita GPU son tests de colcon con la red fija de `test/net_test.yaml`: configuración, topología (colores, adyacencia, empaquetado, orinques), reloj e informe de tiempo real, el subpaso XPBD de referencia (la red se asienta, cajas y un ROV que la empujan sin atravesarla, boyas acopladas) y el agarre de una pinza. Los de la red completa tardan unos minutos:

```bash
colcon test --packages-select stonefish_net && colcon test-result --verbose
```

`build/stonefish_net/gpu_check` (se compila con el paquete, no es un test de colcon: necesita GPU) comprueba que los shaders den lo mismo que la referencia (`gpu`, también con el agarre), mide el coste por tick (`bench`) y la vibración de la red (`shake`). Lánzalo desde la raíz del workspace, dentro de `humble-gpu` y con las variables de PRIME:

```bash
build/stonefish_net/gpu_check gpu src/stonefish_net/test/net_test.yaml
build/stonefish_net/gpu_check bench src/stonefish_net/config/net.yaml   # ms por tick, como en el simulador
build/stonefish_net/gpu_check shake src/stonefish_net/config/net.yaml
```

## Usar otro escenario

Pasa tu `.scn` y la carpeta de sus mallas:

```bash
ros2 launch stonefish_net net_cirtesu.launch.py \
  scenario:=/ruta/mi_escenario.scn \
  simulation_data:=/ruta/mis_datos \
  net_config:=/ruta/mi_red.yaml
```

Lo que necesita el escenario:

1. **`<ocean>`** dentro de `<environment>`. Sin él las boyas no flotan y no hay corriente.
2. **Los looks de la red**, para que tenga sus colores. Si faltan no hay error, pero la red sale gris. Añade este bloque dentro de `<scenario>`; vale aunque el escenario ya tenga su propio `<looks>`:

   ```xml
   <looks>
       <look name="net_twine" rgb="0.00 0.00 0.00" roughness="0.9"/>
       <look name="net_rope" rgb="0.00 0.00 0.00" roughness="0.9"/>
       <look name="net_lead" gray="0.0" roughness="0.6" metalness="0.3"/>
       <look name="net_concrete" gray="0.6" roughness="1.0"/>
       <look name="net_buoy" rgb="1.0 1.0 1.0" roughness="0.4" texture="$(find stonefish_net)/data/buoy/buoy.png"/>
   </looks>
   ```

   Para no modificar un escenario de otro repo, crea un envoltorio: un `.scn` con un `<include>` del original y este bloque, como hace `scenarios/bluerov_cirtesu_net.scn`.
3. **Nada de materiales.** Los de las boyas y los bloques (`Polyethylene` y `Concrete`) los crea el simulador si el escenario no los tiene. No pongas un bloque `<materials>` en el envoltorio: Stonefish solo lee el primero que encuentra, y taparía el del escenario original.

Y en tu copia de `net.yaml`, ajusta a la escena:

- `pose`: dónde va la red.
- `mooring.floor_z`: la profundidad del suelo. La red no choca con mallas cóncavas (como el tanque): el suelo es un plano a esa profundidad.
- `mooring.wall_y`, si la atas a las paredes: tampoco choca con ellas. Sin fondeo abajo, la corriente lleva aguas abajo la parte inferior de la red (en CIRTESU, con 0,2 m/s, ~1,5 m), así que deja sitio hasta la pared de aguas abajo. Pon también `wall_z` bajo el agua (z ≥ 0): el solver no tiene superficie libre y trata como sumergido todo nudo.
- `robot_contact`: los robots de tu escenario, o `[]` y deja que lo haga `robot_contact_auto`.

## Problemas frecuentes

**Va lento.**
1. Comprueba que está compilado en Release: `grep CMAKE_BUILD_TYPE build/stonefish_net/CMakeCache.txt`.
2. Si hay otros ordenadores con ROS 2 en la misma red, aísla el tuyo **antes de lanzar**: `export ROS_DOMAIN_ID=<número propio>` (o `export ROS_LOCALHOST_ONLY=1`). Si no, el simulador manda las imágenes de las cámaras a los nodos de otros equipos por la red y se atasca. También se mezclan los topics de los dos simuladores.
3. Prueba con `spike.yaml`. Si así va fluido, el cuello de botella es la física de la red. Con `gpu`, mira en el diagnóstico la línea `tiempo real` (ver [Tiempo real](#tiempo-real)): baja `solver.substeps` o sube `net.bar`. Con `bullet`, activa `sleep` (ver [Baldosas dormidas](#baldosas-dormidas)), sube `net.bar` o baja `solver.iterations`.

**El simulador se cierra al arrancar.**
- Si dice `configuración de la red '…' no válida: <campo>: <motivo>`, corrige ese campo del YAML.
- Si dice `la red no puede calcularse en la GPU: …` o `solver.backend gpu …`, la GPU no está disponible (sin ventana, OpenGL menor de 4.3, un shader que no compila: el mensaje trae el log). Usa `solver.backend: bullet`. Nunca cambia a Bullet por su cuenta.
- Si se cae después de cargar el escenario, la pila de llamadas queda en `~/.ros/log/stonefish_net_crash.log`. Si en la pila aparece `ScenarioParser` / `LoadGeometryFromFile`, el fallo está en el escenario, no en la red. Por ejemplo, una malla `.obj` cuya línea `mtllib` apunta a un `.mtl` que no existe. Más detalle del parser en `~/.ros/log/stonefish_ros2_parser.log`.

**La red sale gris.** Faltan los looks en el escenario (ver [Usar otro escenario](#usar-otro-escenario)).

**Comprobar los materiales.** Al arrancar, las líneas `Material … created` deben empezar por los del escenario base (`Material Neutral (0) created.` en el del CIRTESU) y acabar con `Polyethylene` y `Concrete`. Con clic izquierdo sobre un objeto, el panel *SELECTION INFO* muestra su material.

## Estructura

```
config/       net.yaml (red del tanque), spike.yaml (red pequeña de prueba) y spike_fine.yaml (red de bar 1,5 cm)
data/buoy/    mallas, textura y ojal de la boya (generados con tools/buoy_asset.py)
launch/       net_cirtesu.launch.py
scenarios/    bluerov_cirtesu_net.scn: el tanque del BlueROV más los looks de la red
include/, src/
  core/       net_core: topología, hidrodinámica, contacto, baldosas, colores de tramos, agarre con pinzas, el
              vocabulario del solver XPBD (xpbd.hpp) y su subpaso de referencia (xpbd_ref), sin ROS ni Stonefish
  gl/, gpu/   net_gpu: contexto de OpenGL de cálculo y el subpaso XPBD en la GPU (sin ROS)
  *.cpp       adaptador a Stonefish: NetSolver con GpuSolver y BulletSolver, NetEntity (dibujo), NetForces
              (enganche al tick), SolidContacts (cuerpos con contacto propio), RobotContact (su contacto en la
              CPU con Bullet), NetSleep (baldosas dormidas), NetWrenches (fuerza sobre los robots y las pinzas),
              NetSimulationManager (monta la escena) y el ejecutable
shaders/      compute shaders de la red en GPU (bindings y constantes los define GpuXpbd.cpp al compilarlos)
test/         tests de colcon, su red (net_test.yaml) y las escenas de prueba comunes con gpu_check (support/)
tools/        gpu_check (comprobaciones del solver en GPU) y buoy_asset.py
docs/         diseño y planes
```

## Limitaciones

- El contacto propio aproxima cada enlace o sólido por **una caja redondeada**: una pieza muy irregular toca la red con su envolvente. Para que sea más ajustada, mídela a mano en `robot_contact` (solo enlaces de robot).
- El suelo es un **plano** a `mooring.floor_z`: un fondo irregular no se tiene en cuenta.
- La red no se rompe ni se corta durante la simulación: los agujeros se generan al arrancar.
- Licencia GPLv3, como Stonefish.
