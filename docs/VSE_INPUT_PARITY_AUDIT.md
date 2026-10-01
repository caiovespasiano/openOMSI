# VSE Input Parity Audit — árvore de input como fonte da verdade

> **FASE 1 — AUDITORIA. Nenhum código alterado para este documento.**
> Metodologia idêntica à auditoria anterior: `input físico → binding → estado →
> sistema consumidor → processamento → saída`, com `arquivo:linha`.
> Origem (verdade): `D:\Programação\Engine Simulador` (C++17).
> Alvo: `F:\Programação HD\openOMSI` (Rust 1.85).
> Restrição ativa: **NÃO MEXER NA VERSÃO ANDROID** (`android.rs`, `launcher/mobile.rs`,
> `touch.rs`, manifest, targets `android`, comportamento mobile).
> Doc anterior (pré-requisito lido integralmente):
> `docs/VSE_TO_OPENOMSI_CAMERA_STEERING_PORT.md`.

---

## 1. Árvore VSE (fonte da verdade)

### 1.1 Drive — `W/S/A/D` (CONFIRMADO)

- Evento físico `WM_KEYDOWN/UP` → `src/editor/EditorWindow.cpp:598,602` →
  `EditorViewportHost::OnKeyDown/OnKeyUp`.
- `src/editor/EditorViewportHost.cpp:3419-3434`:
  `W→SetKeyThrottle(true)`, `S→SetKeyBrake(true)`, `A→SetKeySteerLeft(true)`,
  `D→SetKeySteerRight(true)` (cada um com `return`).
- `src/editor/EditorViewportHost.cpp:4687-4702` (`OnKeyUp`): limpa simetricamente
  (`SetKeyThrottle/Brake/SteerLeft/SteerRight(false)`).
- Estado: `src/editor/SimulationPreviewHost.hpp:147-150`
  (`m_keyThrottle/m_keyBrake/m_keySteerLeft/m_keySteerRight`).
- Consumo (polling 100 Hz, híbrido OR evento+nível):
  `src/editor/SimulationPreviewHost.cpp:1826-1837`
  (`GetAsyncKeyState('W'/'S'/'A'/'D') & 0x8000 || m_key*`, anulado por mouseDrive).
- Saída: `:1846-1858` (`pedal_throttle/brake_held`, `analog=-1.0`);
  `:1970-1994` (`steering_input ±= active_rate*kFixedDt`, clamp `[-1,1]`);
  `:2117-2126` (`Flush`, `UpdateIgnitionKeyHold`, `m_manager->Update(kFixedDt)`).
- Via `SET_THROTTLE/BRAKE/STEERING` (`src/api/CommandProcessor.cpp:174-198,258-260`,
  montada em `SimulationPreviewHost.cpp:1763-1797 DispatchDriverCommands`) existe mas
  **⛔ SEM CHAMADOR no repo** — via ativa no Play é escrita direta em `VehicleState`.

### 1.2 Câmbio — autoridade pelo tipo de transmissão

- Verdade do tipo:
  `src/parsers/VehicleConfig.hpp:415-419`
  (`TransmissionConfig.type = automated_manual|manual|automatic|dct`).
- `src/systems/powertrain/Drivetrain.cpp:517-535`: `automatic|dct|automated_manual →
  transmission_manual=false`; `manual → transmission_manual=true`.
- Autoridade (`Drivetrain.hpp:110-138`, `Drivetrain.cpp:2335-2395`):
  `HasClutchPedal` = `type=="manual"` (TAB só então); `HasReverse` (vetor não vazio);
  `ForwardGearCount` (tabela manda); `IsAutomated` (`automated_manual`);
  `ToggleAutomatedOverride` (só automatizado); `DriverHasGearAuthority`
  (manual sempre; automatizado só com override; automatic/dct nunca);
  `AcceptsCandidateGear` (default false).
- Via A (evento): `EditorViewportHost.cpp:3435-3450`
  (`VK_SHIFT→SuggestGearUp/ShiftGearUp`, `VK_CONTROL→SuggestGearDown/ShiftGearDown`);
  `SimulationPreviewHost.cpp:939-967` (`ShiftGearUp/Down` com `ws_gear_delta ±1`;
  se `!transmission_manual`: `R(-1)→N(0)`, `N(0)→D(1)` e inverso — relação
  `GearDown <- N -> GearUp` confirmada **só aqui**);
  `SuggestGearUp/Down:969-977` (`opticruise_target_gear ±1 clamp [1,12]`);
  `SelectDirectGear:979-983` (`ws_gear_request`, `1..12`), chamado só em
  `EditorViewportHost.cpp:3451-3454` para `'1'..'6'`.
- Via B (polling `PollDriverGearInput`, 1x/frame fora do substep):
  `SimulationPreviewHost.cpp:2156` (chamada), `:2838-2922` (definição),
  `Edge(vk):2932-2950` (só bit `0x8000` + rising, ignora toggle `0x1`).
  Com autoridade: `1..7+R` via `RequestDriverGear` (`:2868-2892`, recusa se `g>nMarchas`);
  sem autoridade: `VK_LSHIFT/VK_LCONTROL` movem seletor DNR com clamp (`:2903-2910`);
  automatizado: `Z` edge alterna override (`:2855-2863`); candidata: `RSHIFT/RCTRL`.
- `N` em `OnKeyDown` Play (`:3544-3547`) é **buzina** (`SetAirHorn`), NÃO Neutral;
  `R` sem binding em `OnKeyDown` Play; Neutral/Reversão por evento só via stepping DNR
  ou `SET_NEUTRAL/SET_DNR` (`CommandProcessor.cpp:199-223,267-276,376-380`,
  `N` domina `D/R` no mesmo flush).
- **⛔ EVIDÊNCIA INSUFICIENTE**: tabela marcha-a-marcha `1-7` com ratios/gates no
  caminho input→UI; `OnKeyDown` expõe `1..6`, `Poll` expõe `1..7+R`, `SelectDirectGear`
  aceita `1..12`; nenhum binding unificado `N=Neutral/R=Reverse/1-7`.

### 1.3 Setas — câmera F1/F2, NÃO steering (CONFIRMADO)

- `EditorViewportHost.cpp:3520-3523`: setas dão `return`
  (`Handled continuously in SimulationPreviewHost::Update`). Sem `SetKeySteer`.
- `SimulationPreviewHost.cpp:2162-2207`: nível `GetAsyncKeyState(VK_LEFT/RIGHT)`,
  edge `!m_prevKeyLeft/Right`; F1 `SwitchDriverCamera(-1/+1)` (`:2175-2182`),
  F2 `SwitchPaxCamera(-1/+1)` (`:2183-2190`), F4 `OnPan(±10px)`, senão F3
  `OnCameraPan(±1.0)` (`:2191-2204`); escreve `m_prevKey*` (`:2206-2207`).
- `WrapCameraIndex:1246-1252` (circular); `SwitchDriverCamera:1566-1589`
  (ease mesma unidade, instant cross-unit); `SwitchPaxCamera:1591-1599` (instant).
- Up/Down (`:2209-2220`): F4 `OnPan(0,±10)`, não-interior `OnCameraPan(0,±1)`;
  **em F1/F2 nada fazem** — comentário `Up/Down tilts head` (`:2158-2161`) vs código:
  ⛔ GAP comentário-vs-código.
- Space: `EditorViewportHost.cpp:3491-3494` (`SPACE→NotifySpaceKey(true)`),
  `:3496-3498` (`BACK/HOME→ResetCameraCenter` direto);
  `SimulationPreviewHost.cpp:1601-1606` (edge `!m_prevKeySpace`),
  `:2167-2169` (polling `0x8000|0x1`);
  `ResetCameraCenter:1608-1631` (`chaseYaw=0/pitch=12`; `home=m_principalDriverCamIdx`
  de `[set_camera_std]`, **NÃO 0**; já-driver→ease, senão instant).

### 1.4 Mouse VSE

- `EditorWindow.cpp:295-305` (`forwardSceneMouse`, `RemapClientToScene`);
  `:399,503,527,581` (`IsOverUiChrome` consome antes); `:559-584` (move/wheel).
- `EditorViewportHost.cpp:2238-2494` (`OnLMouseDown`): F4 FreeCam sem shift/alt →
  `ScreenPointToTerrain→hit`, senão fallback `Z=0` (`-originZ/dirZ`) →
  `SetOrbitTargetSmooth` (`:2264-2279`); F1/F2 com mesh → ray→`invW`→Möller por
  triângulo (`:2390-2409`) com filtro viewpoint/visible/mouseEvent →
  `BeginCockpitDrag` ou `TriggerMouseEvent Down` (`:2281-2491`); F3 → `return`
  (`:2311-2313`); editor → gizmo/path/entity. **LMB nunca orbita global.**
- `OnLMouseUp:2626-2650`, `OnRMouseDown:2652-2671`
  (Flycam cancel; Play+mouseDrive→`SetMouseDriveEnabled(false)`),
  `OnMMouseDown:2678-2812` (guard `F1Mmb+180ms:2685`; `Alt+MMB:2691-2811`
  mesh Möller→esfera→`Z=0`→`SetOrbitTargetSmooth`), `OnMMouseUp:2814-2826`.
- `OnMouseMove:2828-2928`: mouseDrive `nx/ny` (`:2837-2857`); drag cockpit
  (`:2861-2865`); Play F4 (`:2901-2912`: `Ctrl+MMB/Alt+RMB→PrecisionDolly`,
  `Shift+MMB/RMB→Pan`, `MMB/Alt+LMB/RMB→Orbit(dy,dx)`); interior F1
  `MMB→OnInteriorOrbit`, `RMB→OnInteriorZoomDrag` (`:2919-2922`); F3
  `MMB||RMB→OnChaseOrbit` (`:2924-2925`).
- `OnMouseWheel:3131-3150`: interior→`return` (só RMB-drag); F4→`m_camera.OnZoom`;
  F3→`m_previewHost.OnZoom`; editor→`m_camera.OnZoom`.
- `SimulationPreviewHost`: `OnInteriorOrbit:1531-1542`
  (`0.35*0.70`, `zoomStab 1-0.25z`, `yaw+=dxk`, `pitch-=dyk`, `±90/-35..+35`, só F1);
  `OnInteriorZoomDrag:1544-1556` (F1/F2, `dy>0` in, `364px`, intent `0.70`);
  `OnChaseOrbit hpp:277-282` (`yaw-=dx*0.35`, `pitch+=dy*0.35`, `-10..75`);
  `OnZoom:1520-1529` (ignora interior; `dist-=delta*1.5`, `4..40`);
  `EditorCameraController.cpp:62-139` (`OnOrbit/OnPan/OnZoom/Dolly` genéricos).
- `PathTool.cpp:985-1057` (`ScreenPointToTerrain`: marcha
  `dt=clamp(0.75/horiz,0.05,2.0)`, `SampleGround`, 14 bisseções; `false` sem fallback —
  fallback `Z=0` só nos callers). `PathTool:2586` (`Z=0` não é ground).
- Repeat WM sem filtro exceto Space/O/Z-edge: ⛔ sem debounce geral.

### 1.5 Prioridade/contexto VSE

- `EditorWindow::HandleMessage`: `Ctrl+Z/Shift+Z/Y` (undo/redo) antes de `OnKeyDown`;
  `IsOverUiChrome/RmlChromeReady` (botão/move/wheel) antes de `forwardSceneMouse`.
  **UI vence câmera/veículo.**
- `OnKeyDown:3401-3414` (`Ctrl+Shift+B`, `ESC`) antes do Play; `:3417-3648`
  (`IsActive`: condução→motor→câmera F1-F4/Space→sinalização→luzes) com
  `return // Consume unhandled during Play` — editor inalcançável em Play;
  `ESC:3643-3646` (`StopVehiclePreview`); fora Play `:3651+` (F5, flycam WASDQE).
- `Update`: física WASD → câmera setas/space → animação → pose; `SetCameraMode`
  (`:1457-1472`) descarta look/zoom.
- `MouseDrive O:3548-3557` (edge `m_prevKeyO`), `RMB:2652-2670` desliga,
  `StopVehiclePreview:3332` desliga.

---

## 2. Árvore openOMSI ANTES (atual, verificada)

### 2.1 Fontes

- `crates/omsi-app/src/stock_keys.rs:8-137` (`STOCK_KEYS`, espelho `keyboard.cfg`);
  `:15-19` views (`59/0 driver, 60/0 pax, 61/0 outside, 62/0 map, 87/0 ego`);
  `:22-25` reset (`46/0 direction, 57/0 all=Space`) + `interiorcam ∓ (203/205)`;
  `:47-53` throttle/brake/amplify/clutch/steer (`72/78/80/15/75/76/77 + Shift`).
- `crates/omsi-content/src/input.rs:7-106` (`KeyBinding`, `KEY_HOLD/SHIFT/CTRL/ALT`,
  `chord()`, `with_game/vr_defaults`); `crates/omsi-app/src/startup.rs:70-100`
  (qual `keyboard.cfg` vale; `own_keys/own_shift` vs stock).
- `crates/omsi-app/src/keys.rs:6-120` (DIK; `W=17,S=31,A=30,D=32,Space=57,
  Up=200,Left=203,Right=205,Down=208,F1=59..F4=62`).
- `crates/omsi-app/src/cli.rs:52-57` (`--drive-keys simple|wasd|arrows|omsi`,
  default `simple`; `simple`=WASD+setas, `wasd`=só WASD, `arrows`=só setas,
  `omsi`=só Shift+numpad).
- Estado (`app.rs:82-224`): `keys:85`, `mouse_look:88`, `mouse_drive:130`,
  `mouse_steer:133`, `mouse_edge:138` (ratchet VSE), `mouse_pedals:140`,
  `mouse_kmh:142`, `vse_free_goal:143`, `game_keys:183`, `own_keys:187`,
  `dragging:202`, `drag_delta:210`, `look:213`, `view_looks:217`, `cam_blend:220`,
  `view_zoom:223`, `orbit:224`.
- Fluxo (`input_script.rs:58-532` `on_key`; `app_events.rs:80-209` eventos;
  `:2339-2473` `wheel()/left_button`; `:1271-1464` frame; `:620-687` mouse-drive;
  `player.rs:121-141` `fallback_action`; `omsi-sim/input.rs:9-104` `EngineAction`,
  `KeyboardAxes`, `set/release_all`).

### 2.2 Tabela Input|Contexto|ANTES|estado→consumidor→saída

| Input | Contexto | ANTES (arquivo:linha) | Cadeia |
|---|---|---|---|
| `W/S/A/D` | drive `simple/wasd` | `player.rs:135-138` + `input_script.rs:495-501` | `fallback_action` → `axes.set(Throttle/Brake/SteerL/R)` → `KeyboardAxes` → throttle/brake/steer. `view!=free && !shift && !ctrl+alt`; `own_keys` protege rebind |
| `W/S/A/D` | `arrows/omsi`, free/foot/menu/editor | `input_script.rs:482-492`, `cli.rs:52-57`, `app_events.rs:1418-1419`, `on_foot.rs:833-835` | free-fly / foot-walk / menu-nav / chooser. `Shift+WASD` → veículo (`kw_wipermode_up 17/0`, `view_toggle_viewpoint 31/0`, `automatic_D 32/0`, `player.rs:113-116`) |
| Setas | drive `simple/arrows` | `player.rs:131-134` + `input_script.rs:495-501` | `Throttle/Brake/Steer` (idem WASD). `wheel_steering()==true` → viram glance (`app_events.rs:1307-1330`) |
| Setas | câmera | `Ctrl+Left/Right→interiorcam ∓:252-259`; `Ctrl+Alt+setas→mirror:1281-1298`; `Alt+IJKL look:1331-1344`; `input_script.rs:223,227` (`plain_arrow` nunca dispara `view_interiorcam_*` do cfg) | `cam_choice`, `mirror_offsets`, `look`. `free`: `yaw ∓60dt/pitch ±40dt:1452-1463`; `foot`: `±90/±60:776-787` |
| `Space` | cockpit | stock `57/0 view_reset_all:22-23` → `game_action:2245-2261` | `look=(0,0)+zoom.clear+orbit=DEFAULT+cam_choice=(0,0)` (≡ `camera_std`, correto) + menu/chooser `Space=Enter` |
| `Space` | free/foot | `app_events.rs:1430-1431` (`E‖Space` sobe); `on_foot.rs:734-741` (jump); `input_script.rs:512` (`fly_key`) | `Camera.position` / `Foot.vz` |
| `LShift/LCtrl` hold | global | `input_script.rs:121-125` + `input.rs:43-44` | Hold set. Portas `Shift+1..9:335-362`; veículo `Shift+WASD:503-529`; mapa `Shift+M:410-417`; `Shift+N:418-434` navigator; free boost `1436-1440`; `Ctrl+setas` câmera/gear/mirror; `Ctrl+Shift+G` apeadeiro; `Ctrl+S` editor-save vs `Alt+S` quicksave |
| `N/R/1-7` | veículo | stock `kw_s_N/automatic_N 49/0`, `kw_s_R/automatic_R 19/0`, `kw_s_1..6/automatic_1/2`, luzes/bilheteria (`stock_keys.rs:72-80`) via `p.key:514-530` | Triggers enviados sempre; bus filtra por trigger existente. `Shift+N` (navigator) e `Shift+R` (radio) consomem antes; `Shift+1..` (portas) consome antes de gear |
| `Ctrl+Up/Down` | câmbio manual | `input_script.rs:245-249` → `shift_gear:2310-2348` | `kw_s_plus/minus` senão `kw_s_N/R/1..` via `antrieb_getr_aktugang` + `Clutch=1.0` |
| `LMB` | global | `app_events.rs:194-209,2407-2473` → `input_script.rs:920-1056` | `placing_click` → menu → `editor_mouse` → navigator/teleport → chat → `free&&!ego&&!shift: vse_free_goal` (Alt tenta `surface_hit/body_hit` antes de `vse_ground_hit/Z=0:978-1018`) → cockpit `click/drag/html` |
| `MMB` | global | `app_events.rs:158-168` (`mouse_look=pressed`) + `:2291-2292` (`look_by 0.15`) | `look` (free `yaw/pitch`; outside clamp `-10..75`; driver/pax `±90/-35..35`). Não toca `mouse_drive`. Ignorado com mapa aberto |
| `RMB` | global | `app_events.rs:124-156` (`mouse_drive`→off+`keep_wheel`, senão `mouse_look=pressed`; VR `zoom`) | Mesmo look do MMB + desliga mouse-steer. Foot/free idem |
| `Wheel` | global | `app_events.rs:2339-2404` (editor→placing→menu→map→chat→hover-switch→view) | `editor_wheel` / `placing_wheel` / `menu_wheel` / `map_wheel` / `chat.wheel` / `p.wheel(-amount*40)` (switch tem prioridade) / senão `outside+ctrl→zoom_by`, `outside→orbit-=1.5a`, `driver/pax→zoom_by`, `free/foot→zoom_by` senão dolly |
| `Alt+MMB` | — | **0 hits** (grep `Alt.*MMB|MMB.*Alt`) | Inexistente. Mesh-pick é `LMB+Alt em free:979-1000`. MMB+Alt = look normal |
| `F1-F4` | views | hardcoded `input_script.rs:376-383` + `game_action:2195-2213` + stock `59-62/0` | `driver/pax/outside/free(detached respawn chase, VSE_CHASE_FOV)`; `cam_choice`, `look_key`, `sync_view_look`; `on_foot` F1/F4 próprios |
| `Q/E`, `F5-F12` | fly/bus/misc | `app_events.rs:1430-1435` fly; `p.key(16/18)` veículo (`IBIS_vor/bateria`); `placing Q/E/R` consome antes; `F9` career, `F11` log-pose (conflita `view_set_ego 87/0`→foot), `F12` screenshot vs `cp_schalter_kinderwagen 88/0` | Conforme contexto; `fly_key`/`foot_key` bloqueiam `p.key` |

### 2.3 Prioridade openOMSI (`on_key:58-178`)

`Esc/map` → menu-start → chat (`V`,`/`, typing consome tudo) → `keys.insert` →
VR → key-up `door_off` → `placing_key` → `menu_key` → `editor_key` → `Esc` abre menu →
tutorial → `foot_key` (walker consome quase tudo) → `[game]` cfg (`pressed&&!repeat`,
`chord`, `own_keys`, `plain_arrow` protege interiorcam) → hardcoded edge
(`O/Ctrl+setas/Ctrl+Alt+setas/...`) → extras (`Z/X/C/I/F1-F4/...`) →
`fallback_action→axes.set` (não-free, sem shift/ctrl+alt; `wheel_steering` força
`arrows|omsi`) → `p.key(scan,chord)` (filtrado por `driving/fly_key`).

### 2.4 Estado vs evento / mouse (openOMSI)

- Hold: `keys:HashSet` (`:121-125`), lido por frame (`:1279-1463`, `on_foot:775-845`),
  limpo em `suspended/Focused(false)` + `axes.release_all()`.
- Edge: `pressed && !repeat` (`:212,235,314`); sem `m_prevKeyO` (grep 0) —
  equivalentes: `blinker_key_state:714-750`, `cam_before_special:2228-2234`,
  `door_key_triggers:146-162`, `arrow_glance:1311-1323`, `vse_free_goal`,
  `cam_blend`, `chat.swallow`, `mouse_steer fade + mouse_edge ratchet`.
- Repeat: `event.repeat` (winit `:103-119`) ignorado p/ edge; `plugin_keys`, chat,
  `foot jump` exigem `!repeat`.
- Mouse: `MouseInput Pressed/Released (L/R/M:124-209)` + `left_button/on_left` +
  `MouseMotion dx,dy` (`look_by*0.15` se `mouse_look` senão `mouse_past_edge`) +
  `dragging/drag_delta` → `drag_frame→p.drag` + modifiers (`keys.contains`,
  `chord()`) + cursor físico (`move_cursor`).

---

## 3. Conflitos (ANTES vs VSE) — lista de eliminação

1. **Setas dirigem** (`player.rs:131-134` + `input_script.rs:495-501`, presets
   `simple/arrows`): VSE=setas **nunca** dirigem (`3520-3523` + `Update`). Eliminar a
   rota setas→`EngineAction`, preservando `Ctrl+setas` (câmera), `Ctrl+Alt+setas`
   (mirror), `Alt+IJKL`, free/foot.
2. **Wheel global** (`app_events.rs:2393-2399` `driver/pax/free/foot zoom_by` +
   `:2370-2385` hover-switch): VSE=wheel **só F3** (`OnMouseWheel:3131-3150`;
   interior `return`). Gatear zoom por view; switch continua com prioridade
   (interação > câmera, como UI>veículo no VSE).
3. **RMB = look global** (`:124-156,2291-2292`): VSE=`RMB+drag vertical` é **zoom F1**
   (`OnInteriorZoomDrag`), `RMB` em F3 orbita com MMB, `RMB` em Play+mouseDrive
   desliga drive (já ok). Separar `Right` de `Middle` no motion.
4. **MMB = look genérico** (`:158-168`): VSE=`MMB+drag` é **orbit por modo**
   (`OnInteriorOrbit/OnChaseOrbit/OnOrbit` com sens/sinais/clamps próprios).
   Roteamento por view já existe em `look_by` (clamps VSE); falta sens/sinal por
   botão e `Alt+MMB` dedicado.
5. **`Alt+MMB` inexistente** (grep 0): VSE tem caminho mesh→esfera→`Z=0`
   (`OnMMouseDown:2691-2811`). Hoje só `LMB+Alt em free`. Criar rota `Middle+Alt`
   (mesh via `surface_hit/body_hit` → ground → `Z=0`) sem tocar no LMB-interação.
6. **LMB em free retargeta sempre** (`:978-1018`, qualquer clique sem `Shift`):
   VSE=LMB é **interação**; retarget F4 é `LMB sem shift/alt` **só em FreeCam**
   ( Play) vs cockpit-drag em F1/F2 — e no openOMSI `free` não tem switch sem
   retargetar. Exigir `Alt` (ou manter sem-modificador só com `vse_free_goal` sem
   roubar `click`?) — decisão §4.
7. **WASD vs OMSI** (`W=wiper,S=viewpoint,D=gear-D` no stock, acessíveis com `Shift`
   em `simple/wasd`): VSE=WASD **sempre** dirige em Play. Manter `Shift+WASD`→veículo
   é compatível (VSE não define `Shift+W`), mas `arrows/omsi` presets deixam WASD
   fora do drive — VSE exige WASD no drive em desktop. Travar preset efetivo em
   `wasd|simple` no caminho de drive (sem tocar CLI/mobile).
8. **Glance só com volante** (`:1307-1330` exige `wheel_steering()`): VSE=setas são
   câmera; sem volante as setas hoje dirigem. Ao eliminar (1), habilitar glance/look
   por setas sem volante (só driver, com retorno exp como já existe).
9. **F4**: stock `view_set_map 62/0` sugere mapa, mas código já faz respawn chase
   (correto VSE). Mapa real é `Shift+M`/navigator. Sem mudança funcional; só nomear
   no doc para não "corrigir" errado.
10. **F11 duplo** (físico=log-pose `:435-458` vs cfg `view_set_ego 87/0`→foot):
    fora do escopo VSE pedido (ego/foot não são F1-F4/drive); **não tocar**
    (evita regressão; foot é usado por mobile/on-foot).
11. **Câmbio incondicional** (`N/R/1-6` via `p.key` sempre; `Shift+N/R/1..` consumidos
    por navigator/radio/portas antes): VSE=condicional por transmissão
    (`DriverHasGearAuthority`, stepping DNR `LCTRL/LSHIFT`, `N/R/1-7` manuais).
    Teclado precisa de `gear_up/down` condicional + supressão `N/R/1-7` conforme
    `manual_gearbox()` — sem tocar em `touch gear:289-311` (mobile).
12. **Duplos caminhos**: `O`/`K` (cfg `[game]` + hardcoded, ambos `game_action` —
    benigno); `P` (pause cfg + hardcoded + menu + foot passthrough);
    `Space` (reset cfg + fly-up + jump + menu-Enter — separar por `view`);
    `Q/E` (placing vs fly vs veículo); `I/K/J/L` (editor vs `Alt+`look vs bus);
    `C/V` (editor vs blinker vs chat); wheel-sobre-switch roubando câmera;
    `mouse_drive` vs `mouse_look` (RMB desliga drive); `bus_view` gate;
    controller `gear_up/down` → `shift_gear` competindo com `p.key`.

---

## 4. Decisões de implementação (pós-auditoria, antes do código)

- D1 Setas: remover `Arrow*→EngineAction` do `fallback_action` (todos os presets);
  setas passam a ser **sempre câmera** (`Ctrl+Left/Right` já é interiorcam;
  setas puras em driver/pax = `view_interiorcam ∓` = `SwitchDriver/PaxCamera`;
  em `outside` = orbit glance; em `free/foot` = movimento atual). `Ctrl+Up/Down`
  (gear manual) e `Ctrl+Alt+setas` (mirror) intactos.
- D2 Wheel: só `outside` altera `orbit` (`±1.5/notch`, VSE `1.5*delta`);
  `driver/pax` **não** recebem `zoom_by` do wheel (VSE interior `return`);
  `free/foot` não recebem `zoom_by` (VSE F4 `m_camera.OnZoom` = distância orbital,
  não FOV — aqui `free` é fly: wheel vira dolly `fwd*4.0` como o ramo `else`);
  `Ctrl+wheel` em `outside` mantém telefoto (`zoom_by`); prioridades
  editor/placing/menu/map/chat/hover-switch intactas (interação > câmera).
- D3/D4 Mouse: `MouseMotion` com `mouse_look` roteado por botão —
  `Middle→look_by` (orbit, todas as views), `Right→ F1(driver): zoom-drag vertical
  VSE (`vse_interior_zoom`), F3/F4/free: look (orbit)`; `RMB` pressed continua
  desligando `mouse_drive` (VSE `OnRMouseDown`). Registro de qual botão segura o
  look (`mouse_look_btn: Option<MouseButton>`) em vez de bool puro.
- D5 `Alt+MMB`: nova rota dedicada — `Middle pressed + Alt` → mesh
  (`surface_hit/body_hit`) → `vse_ground_hit` → `Z=0` → `vse_free_goal`
  (só `free`, sem `Shift`; fora de `free` consome sem efeito). `LMB+Alt em free`
  preservado como equivalente atual.
- D6 LMB free: exigir `Alt` para retarget? **Não** — VSE `OnLMouseDown` em FreeCam
  retargeta com LMB **sem** modificador (`:2263`, só exclui shift/alt). O conflito
  real (§3.6) é que `free` aqui também é fly sem switches; manter LMB-sem-Shift
  como retarget (VSE-literal) e documentar que interação em `free` é via
  `html_object_click`/editor (inalterado). Nenhuma mudança neste item além de
  somar a rota `Alt+MMB`.
- D7 Drive: `fallback_action` WASD intacto; presets `arrows|omsi` deixam WASD fora
  do drive por design CLI — documentado como divergência consciente (VSE pede
  WASD; quem roda `arrows/omsi` optou pelo OMSI). Sem mudar CLI.
- D8 Glance: com setas fora do drive, `wheel_steering()` deixa de ser exigido para
  glance de setas em `driver` (retorno exp mantido); `Ctrl` continua excluído.
- D9 Câmbio: `Ctrl+Up/Down`→`shift_gear` intacto; adicionar `LShift=GearUp,
  LCtrl=GearDown` **só quando `!manual_gearbox`** (automático/automatizado,
  relação `down<-N->up` via `shift_gear` que já implementa `kw_s_plus/minus →
  N/R fallback`); `N/R/1-7` passam a ser condicionais: em manual vão ao bus
  (`p.key` como hoje); em automático são suprimidos (exceto `N`→`shift_gear`
  neutro? não — VSE `N` em evento é buzina; neutro vem do stepping; então
  suprimir `N/R/dígitos` em automático). `Shift+N/R/1..` (navigator/radio/portas)
  mantêm prioridade (consomem antes). `touch gear` e `controller gear` intocados.
  Tipo via `program.manual_gearbox()`/`antrieb_number_gears` como `touch.rs:289-311`.
- D10 Sem tocar: `android.rs`, `launcher/mobile.rs`, `touch.rs`, targets android,
  manifest; `F11`/foot/ego; mapa (`Shift+M`/navigator); `F4` respawn (já VSE).

---

## 5. Verificação (critério de saída da auditoria)

Cada linha §2.2 tem cadeia `evento→binding→estado→consumidor→proc→saída` acima; cada
conflito §3 aponta o `arquivo:linha` a eliminar (§4 decide como). Implementação deve
provar p.ex. que `ArrowLeft` não alcança `SteeringLeft` em **nenhuma** rota
(`fallback_action` + `[game]` cfg + controller + `foot_key` + menu) e que
`MMB/RMB/Wheel/LMB` têm um único consumidor por contexto.

---

## 6. Implementação (executada nesta sessão, sem commits)

Decisões §4 aplicadas uma a uma. Nenhum arquivo Android/mobile tocado
(`android.rs`, `launcher/mobile.rs`, `touch.rs`, manifest, targets — `git diff`
prova). Toolchain/deps inalteradas.

### Alterações

- **D1 setas fora do drive** — `player.rs:fallback_action`: removidos os braços
  `Arrow*→Throttle/Brake/Steer` em todos os presets (`wasd` = só WASD;
  `arrows`/`omsi` = sem drive por fallback). `player.rs` sem nenhuma ocorrência de
  `Arrow` (verificado por grep). `input_script.rs:482-492`: preset `wheel` não mais
  desvia setas; `launcher/pages.rs:582`: rótulos (`simple` = `W A S D (arrows are
  cameras)`).
- **Setas→câmera** — `input_script.rs:on_key`: `ArrowLeft/Right` puras em
  `driver/pax` → `view_interiorcam_∓` (edge, wrap circular em `game_action`);
  `Up/Down` puras em `driver/pax` e setas puras em `outside` consumidas (sem
  steering, sem `p.key`). `app_events.rs` frame: `outside` + setas puras = orbit
  (`look.0` wrap 360, `look.1` clamp VSE `-10..75`); `Ctrl+setas` (câmera/gear) e
  `Ctrl+Alt+setas` (mirror) intactos.
- **D2 wheel só F3** — `app_events.rs:wheel()`: `outside` → `orbit ∓1.5/notch`
  (VSE `1.5*delta`), `outside+Ctrl` → telefoto (`zoom_by`); `driver/pax` → no-op
  explícito (VSE interior `return`); `free/foot` → dolly `fwd*4.0` (nunca `zoom_by`
  de FOV). Prioridades editor/placing/menu/map/chat/hover-switch intactas.
- **D3/D4 MMB vs RMB** — `app.rs` + `lib.rs`: novo `mouse_look_btn:
  Option<MouseButton>` (sempre sincronizado com `mouse_look`; limpo em focus-loss).
  `app_events.rs` RMB/MMB: gravam o botão; soltura cruzada não apaga o outro.
  `MouseMotion`: `Right` em `driver/pax` → `zoom_by(dy/16.25)` (VSE
  `dy*0.70/364`, `down=in`, 364 px = full); `Right` fora → `look_by`; `Middle` →
  sempre `look_by` (orbit por modo, clamps VSE já em `look_by`). `RMB` pressed
  segue desligando `mouse_drive` (VSE `OnRMouseDown`).
- **D5 Alt+MMB** — `app_events.rs` MMB: `Alt+Middle` em `free` (sem `Shift`) →
  mesh (`surface_hit`/`body_hit`, Möller via `ray_mesh`) → `vse_ground_hit` →
  `Z=0` → `vse_free_goal` (easing `1-exp(-10dt)` no frame). Fora de `free` cai no
  look normal. `LMB+Alt em free` preservado.
- **D6 LMB** — sem mudança (VSE-literal: LMB sem `Shift` em `free` retargeta;
  `driver/pax` é interação/cockpit-drag; F3 sem pick).
- **D8 glance** — `app_events.rs`: `driver` sempre (com/sem volante), `pax` só com
  volante; clamps VSE (`yaw ±90`, `pitch -35..+35`); retorno exp mantido.
  Clamp geral não-`outside` `±140→±90`.
- **D9 câmbio** — `input_script.rs`: `is_manual_gearbox()` (mesma sonda do
  telefone: `program.manual_gearbox()`); `LShift`→`shift_gear(true)`,
  `LCtrl`→`shift_gear(false)` **só** em automático/automatizado, bus-view, sem
  menu/placing/editor/foot (só consome quando dispara; senão cai no fluxo —
  modificadores de chord preservados). `N/R/1-7` **não** suprimidos: `p.key`
  resolve scan→ação via `bindings` do bus e o script filtra triggers
  (`player.rs:708-736`; manual ignora `automatic_*`, automático ignora `kw_s_*`) —
  condicionalidade por transmissão já é arquitetural, igual ao `touch gear`;
  suprimir quebraria `automatic_1/2` e `cp_licht_*` (mesmos scans).
  `touch gear`, controller `gear_up/down`, `Ctrl+Up/Down` intactos.
- **D7/D10** — sem mudança funcional (documentado em §3.7/§3.9/§3.10).

### Verificação final (§24 do pedido)

- `W→throttle, S→brake, A/D→steer` (`fallback_action`, presets `simple/wasd`);
  `ArrowLeft/Right→interiorcam ∓` (edge, wrap); `Space→reset` (já era
  `cam_choice=(0,0)` ≡ `camera_std`); `LShift/LCtrl→gear↑/↓` (só automático);
  manual `N/R/1-7` via bus (condicional por triggers); `LMB→interação`
  (`free`: retarget VSE-literal); `MMB→orbit`; `RMB+drag vertical→zoom F1`;
  `Wheel→zoom F3` (só); `Alt+MMB→F4 pick`.
- Sem consumidores antigos: grep prova `Arrow∅` em `player.rs`,
  `mouse_look = state` eliminado, `zoom_by(amount)` do wheel só no ramo
  `outside+Ctrl`, `Alt+MMB` com rota própria, `mouse_look_btn` sem stuck
  (focus-loss limpa).

---

## 8. Fase cirúrgica — runtime real (F1/F3/zoom/steering/O, sem tocar F2/renderer)

Relato de teste direcionou esta fase; cada item com causa VSE localizada:

### F1 Left/Right — shake morto na origem

- Causa: o bloco glance do frame (`app_events.rs`) girava a cabeça com seta
  segurada **ao mesmo tempo** que a edge trocava a câmera `.bus` (glide VSE) —
  dois escritores de yaw, o glide vencia mas a briga aparecia como shake.
- Correção: em `driver`, Left/Right segurada não toca mais `look`
  (`pax_wheel` preservado — F2 intacta); Up/Down mantidos (pedido explícito).
  Transição F1 tem um único escritor por seta. `.bus`/FOV/easing intocados.

### F1 Space — retorno eased, zoom junto

- VSE (`SimulationPreviewHost.cpp:1608-1631,1363-1433`): `home =
  principalDriverCamIdx` (não 0), `BeginDriverEaseTo` com `zoom→0` dentro do
  mesmo ease `0.54s` ease-out — zoom desfaz progressivo, nunca teleporta.
- Correção: `view_reset_all_directions` em `driver` abre `vse_zoom_return`
  (look+zoom eased, `vse_reset_blend`, mesma curva/duração do giro); outras
  views (incl. F2) mantêm o reset instantâneo validado.

### Zoom de precisão — matemática VSE de verdade

- VSE (`SimulationPreviewHost.cpp:1531-1556,1435-1441`): drag acumula
  `dy*0.70/364` em `zoomAmount 0..1`, FOV = `base/(1+5.5z)`; sem cursor escondido
  (só `SetCapture` no press, `EditorWindow.cpp:396,500,524` + forma em
  `549,553`); `OnInteriorZoomDrag` cobre F1 **e** F2; F3/F4 não têm RMB-zoom no
  VSE (F3: wheel=distância, F4: `m_camera.OnZoom`).
- Correção: `vse_precision_zoom_step` (multiplicador como `z` visto por
  `m=1/(1+5.5z)` — mesmo drag, mesmo FOV que o VSE; ~2× mais rápido que o
  `zoom_by` antigo, que era a diferença de "intensidade" relatada) em
  F1/F3/F4; **F2 mantém `zoom_by` byte-idêntico** (regra absoluta).
  Wheel continua como estava por contexto.

### Zoom negativo — opção de menu, matemática intacta

- Preservado `*1.6` com os mesmos números; novo `settings.zoom_negative`
  (default `true` = comportamento atual) em `settings.rs` + `game_lists.rs`
  (`zoom_neg`) + `launcher/pages.rs`, mesmo padrão de `driverview_smooth`.
  Desligado, `zoom_by` trava em `1.0`. O passo de precisão nunca passa de 1.0.

### F3 — alvo era o sintoma; o sinal do pitch era a doença

- VSE (`SimulationPreviewHost.cpp:2314-2345,2398-2443`): alvo =
  `(vehX, vehY, pivotZ+1.4)` sobre o veículo (eixos/centro, nunca o chão);
  `off=[D·sinCY·cosCP, −D·cosCY·cosCP, 1.6+D·sinCP]` — câmera ACIMA olhando
  para baixo; `dist=wb*1.5+4`, `yaw=0`, `pitch=12`, `4..40`, FOV 60.
- Causa no openOMSI: `pitch=+12` com `forward.z=+sin(pitch)` (positivo = para
  cima, `omsi-render:404-419`) punha a câmera ABAIXO do centro olhando para
  cima — órbita do chão por construção, não por constante errada.
- Correção: `pitch=-(12+look.1)` (`F3`, clamp `[-75,10]`), `look.1` passa a SER
  o pitch VSE (`P=12+look.1`, drag com o sinal do `OnChaseOrbit`, clamp da
  derivação `[-22,63]`); `dist` inicializada por wheelbase uma vez por ônibus
  (`vse_orbit_wb`, persistência do motorista preservada); alvo segue o
  `camera_outside_center` do `.bus` (SD80 real: `0,0,1.2` ≈ `+1.4` VSE).
- Prova numérica: `vse_chase_pose` + `chase_pose_matches_vse_geometry`
  (off/cam/tgt/dist deduzidos à mão do VSE).

### Steering teclado — escritor único

- Causa: `Player::tick` rodava `axes.update` (OMSI antigo) e somava teclado por
  cima via `max()`; o passo VSE (`vse_keyboard_step`) só existia em teste.
- Correção sem mexer na física: `update()` usa as taxas VSE no caminho default
  (`linear`/`centering`/`old_steering` opt-ins intactos), retorno VSE
  (só rolando), e `mouse_owned` zera a autoridade do teclado (pedais e
  direção — `O+W`/`O+S`/`A/D` não somam mais); `tick` toma o analógico do mouse
  como verdade exclusiva. Teste `steering_returns_like_a_spring` reescrito para
  a semântica VSE (números deduzidos da implementação, não chutados).

### Mouse `O` — paridade + cursor confinado

- VSE (`SimulationPreviewHost.cpp:1097-1137,1996-2040`): `NX=NY=0` neutro ao
  ligar, `Sm` semeado no volante atual, edge/corner/slew `tau 0.12`, ratchet de
  quina, pedais deadzone `0.10`, `SetCapture` nos presses (sem `ClipCursor` em
  lugar algum — cursor físico livre + clamp normalizado).
- Mantido o seeding por fade (equivalente sem teleporte); `mouse_edge`
  (BoostHold) agora zera ao desligar nos 3 caminhos (VSE limpa o latch).
- Exigência além do VSE, implementada como superset documentado:
  `confine_cursor` (`CursorGrabMode::Confined`, desktop-only via
  `cfg(not(android))`, erros ignorados) enquanto `O` ativo; libera ao
  desligar/focus-loss. Coordenadas já eram físicas dos dois lados
  (`CursorMoved` físico × `surface.config` físico), sem mismatch de DPI.

### Divergências restantes (declaradas, não mascaradas)

- `look_by`/`zoom_by` cancelam o retorno do Space (o VSE iniciaria novo ease;
  aqui o usuário assume — transiente de 0.54s, aceito).
- Curva do `zoom_by` (`=/-`, pinch, F2) segue multiplicativa; só o drag de
  precisão usa a curva VSE (F2 congelada por regra absoluta).
- Pivot-Z `tau 0.30` do VSE equivale ao easing do `SpringArm` existente
  (adaptação registrada na auditoria anterior; sem estado novo).
- `Up/Down` em F1 mantêm look (pedido explícito) embora o VSE não incline a
  cabeça por setas (gap comentário-vs-código já registrado do próprio VSE).
- F2: nenhum byte de comportamento alterado (rotas `ZoomLegacy` dedicadas).

---

## 9. Fase final — runtime real (zoom 20%, limites, F3 estável, O, sem tocar F2)

Relato de teste direcionou cada item; causas rastreadas ao VSE ou declaradas:

### F1 zoom 20% mais lento (mandato explícito, valor dado: `0.56`)

- `VSE_F1_ZOOM_INTENT = 0.56` (= `0.70 −20%`); F3/F4 mantêm o auditado `0.70`.
  Curva/clamp/direção/retorno/negativo/Space intactos.

### F1 MMB — base VSE `-35°` + margem de zoom (extensão geométrica declarada)

- O VSE fixa `-35°` (`OnInteriorOrbit`) — sem valor menor correspondente; a
  margem extra vem de `vse_zoom_pitch_margin = half(base) − half(base·m)`
  (0 em repouso → VSE puro; ~+25° no zoom máximo). Teste em 3 pontos.

### F3 refeita sobre pivot estável (não remendada)

- Causa raiz do "orbitar o chão": `pitch=+12` com `forward.z=+sin(pitch)`
  punha a câmera ABAIXO do centro; somado ao `body_rotation()` (pitch/bank da
  carroceria) no centro, a câmera herdava bounce + chão como referência.
- Correção estrutural: `stable_pivot` (só heading; `position.z` já é média
  suave do terreno, bounce vive na atitude — excluída), `pitch=-(12+look.1)`,
  `look.1` = pitch VSE (`OnChaseOrbit`), alvo `.bus` (`0,0,1.2` no SD80 ≈
  `+1.4` VSE), `dist=wb*1.5+4` por ônibus (`vse_orbit_wb`, sem tocar o zoom do
  motorista), `camera_clipped` no mesmo pivot. `SpringArm` segue anti-parede.
- F4 herda a base por construção (snapshot do `outside`); detach/pick/fly
  intactos. Space em F3 preserva a distância (VSE não a zera).

### Drive-head smooth = `driverview_smooth` (default ON)

- `vse_glide_active(setting)` no glide e no retorno do Space; OFF = corte
  instantâneo. Teste ON/OFF + default.

### Steering — escritor único, provado

- Inventário: `update()` (teclas, agora VSE no default; opt-ins OMSI intactos),
  bloco mouse→`analog` (só `O`), `poll` (só gamepad físico), touch (só mobile),
  `keep_wheel` (transferência, não integrador); merge em `tick` extraído para
  `vse_pick_steering` (prioridade mouse>gamepad>teclas, sem soma) + teste.
  Setas sem rota para `axes` (matriz + stock, testados). `mouse_owned`
  congela o teclado (pedais e direção) e dá exclusividade ao analógico —
  `O+W`/`O+S` não somam. Resíduo `<1e-4` é a guarda do próprio VSE, não input
  preso (teste documenta).
- `O`: cadeia verificada elo a elo (toggle→bloco→`p.analog`→`tick`→física);
  `mouse_edge` zerado nos 3 desligamentos; grab `Confined` desktop-only
  (VSE usa `SetCapture` + clamp — confinamento é o superset exigido);
  coordenadas físicas dos dois lados (sem mismatch de DPI); focus-loss libera
  o grab e preserva posições (paridade VSE: sem handler de foco lá também).

### Divergências restantes desta fase

- F2 tem código de zoom/look próprio duplicado em espírito (`ZoomLegacy`) para
  não tocar um byte dela — duplicação consciente por regra absoluta.
- Ganho absoluto do drag (0.15°/px do chamador vs 0.245/0.35 VSE) segue item
  para comparação ao vivo (inalterado, como na fase anterior).
- Retorno do Space cancelado por input novo (transiente 0.54s).

---

## 10. Fase causal — steering que anda sozinho + F3 presa (auditoria + correção)

Sintomas de runtime que contradiziam o fechamento: volante à deriva para um
lado, input brigando com reset, `O` derivando à direita, F3 colada no centro.

### A. Steering — inventário final de escritores (nenhum segundo integrador)

| # | Escritor | Condição | Prova |
|---|---|---|---|
| 1 | `KeyboardAxes::update` (VSE) | teclas, exceto `mouse_owned` | `vse_drive_keys_isolated`, spring reescrito |
| 2 | bloco mouse → `analog` | só `O` + bus-view + sem menu | leitura `app_events.rs:688-761` |
| 3 | `poll` → `analog` | só gamepad físico (deadzone 0.08), `controllers.rs:569-596` | leitura; sem pad = `None`, sem escritor |
| 4 | touch → `p.analog` | só `touch.enabled` (mobile) | `touch.rs:837`, desktop intocado |
| 5 | `keep_wheel` | transferência na saída de `O`, não integração | `player.rs:2046` |
| merge | `vse_pick_steering` | precedência mouse>gamepad>teclas, um valor/frame | `steering_merge_has_one_winner` |

Setas sem rota (`arrows_never_drive`, stock audit, `plain_arrow`);
`linear`/`centering`/`old_steering` são ramos do MESMO `update`, não escritores
extras; `steer_look`/`hands`/`auto_clutch` só leem. Diagnóstico permanente para
pegar deriva ao vivo: `OMSI_TRACE_STEER=<csv>` agora também grava
`<csv>.tick` (`frame,axes,analog,final,owned,keys_lrws`) em `Player::tick` —
rode com a variável e leia qual coluna se moveu.

### B. Keyboard — cadeia final, sem segundo escritor

`A/D → fallback_action → axes.set → update (VSE) → tick merge → physics`.
`mouse_owned` congela o teclado inteiro; `tick` dá exclusividade ao analógico
(`O+W`/`O+S` não somam). Resíduo `<1e-4` é a guarda do próprio VSE.

### C. Deriva à direita no mouse — duas fontes, ambas tratadas

1. `mouse_edge` (BoostHold) sobrevivía ao desligar de `O` → resetado nos 3
   caminhos de saída (toggle, menu, RMB), como o VSE limpa o latch.
2. Cursor podia sair da janela e acumular coordenadas imaginárias →
   `confine_cursor` (`Confined`, desktop-only) enquanto `O` ativo; coordenadas
   já eram físicas dos dois lados (sem mismatch de DPI).

### D. `O` — cadeia VSE, divergências corrigidas

`O → mouse_owned → grab → nx/ny → target → slew tau 0.12 → ratchet →
pedais 0.10 → analog → tick exclusivo → physics`. Divergências contra o VSE
sanadas nesta fase: latch sobrevivente, soma de teclado, cursor livre.
Seeding por fade equivale ao `Sm = input` sem teleporte (documentado).

### E. Menu — inventário steering/câmera (nada escreve steering por fora)

`steering_linear`/`old_steering` (ramos do `update`), `mouse_sens` (escala do
target), `wheel_range/lock` (ganho de gamepad), `ff_*` (saída), `pedal_*`
(curvas), `brake_hold` (pedal), `ctrl_deadzone/off` (filtro), `steer_look`
(só lê), `fov`/`camera_collision`/`head_movement`/`hands` (câmera/figura),
`driverview_smooth` + `zoom_negative` (fases anteriores). Nenhuma é segundo
escritor; nenhuma foi removida.

### F. F3 — onde prenderia, e por que não prende mais

`stable_pivot → camera_look (pose) → camera_clipped (arm) → final`. Auditoria:
`camera_blockers` só retorna cenário (nunca o próprio ônibus); `free_length`
só encurta o braço contra parede/objeto ou ao descer a `GROUND_CLEARANCE`
do solo — com o pitch corrigido o raio SOBE do pivot, sem hit; `SpringArm`
só obedece ao `free` (teste `arm_pins_only_when_free_reports_blocked`:
claro→`want`, bloqueado→pino, livre→retorna). Se F3 colar no centro ao vivo,
o culpado é o `free` — capture com `OMSI_DEBUG_CAMERA=1` (loga o que parou o
braço por frame) em vez de chutar `targetZ`/`offZ`.

### G. Arquivos alterados nesta fase

`player.rs` (trace `.tick`), `camera_arm.rs` (teste de mecanismo),
docs. Nenhuma mudança de comportamento além do diagnóstico e do teste.

### H. Testes

Na validação final abaixo. Nenhum teste removido; 1 adicionado nesta fase
(`arm_pins_only_when_free_reports_blocked`); o trace `.tick` é ferramenta
manual, não teste automatizado.

---

## 11. Fase causal — deriva à direita e função apagada (auditoria + correção)

### Causa raiz da deriva (encontrada no código, não no ganho)

`mouse_past_edge` (acúmulo OMSI de borda, ±2.0) e o ratchet VSE (`BoostHold`)
dividiam o MESMO campo `mouse_edge`: com o cursor na borda, o acúmulo entrava
como `hold` no ratchet e travava `target` em full lock — deriva persistente
para a direita, indistinguível de "input brigando". Estados separados:
`mouse_edge` = latch VSE (só o ratchet escreve), `mouse_past` = acúmulo OMSI
(só `mouse_past_edge` escreve); `target = clamp(boost + past)`. Limpeza nos
3 desligamentos + `about_to_wait`; teste `stale_edge_never_latches_the_ratchet`;
coluna `past` no trace do mouse.

### Função apagada restaurada

Uma edição anterior havia deletado `mouse_past_edge` (compilava por
acidente? não — quebrava; detectado no `check` desta fase). Restaurada
byte-idêntica do HEAD, só redirecionada ao campo próprio.

### Trace de escritores

`<csv>.tick` (`frame,axes,analog,final,owned,keys_lrws`) mostra por frame quem
moveu o volante. Com ele, "gira sozinho" vira leitura de coluna, não hipótese:
coluna `axes` mexendo sem `keys` = integrador/retorno indevido; `analog` sem
`O`/gamepad = dispositivo fantasma; `final` ≠ ambos = merge quebrado.

---

## 15. Diagnóstico profundo — jitter A/D e zona-morta no `O`

Sintomas: A/D com volante "clicando" de um lado para o outro; `O` com zona
morta no centro (VSE: deadzone só nos pedais `0.10`).

### Achado 1 — volante visual sem histerese (`driver.rs`)

`wheel_angle` redecidia o sinal do volante 3D TODO frame (`<=` exato): perto
dos pontos equidistantes as duas opções empatam e o sinal alternava quadro a
quadro — `theta = ±by_var` pulando = clique lateral (mãos junto). Extraído
para `wheel_sign` com margem de 8° (aquisição inalterada) + 2 testes
(aquisição exata; travessia do ponto de empate sem flip; espelho real ainda
vira). Física/render pipeline intocados (só a heurística de descoberta).

### Achado 2 — sem zona-morta no `O` (usuário tem razão no VSE)

`MouseSteerTarget`/`MouseDriveIntent`: direção proporcional desde zero;
`0.10` só em `thr`/`brk`. Teste `mouse_target_has_no_center_deadzone`
(varredura 0→1 estritamente crescente, zero só no centro). Sensação de zona
morta restante = slew `2.5/s` + `tau` (resposta, não trava) ou fade de 1s.

### Descartados com prova

- `steer_vel`: só-escrita (sempre 0.0), nenhum leitor.
- `mouse_owned`/`axes.set`/repeat: idempotentes; setas sem rota (testado).
- Cursor confinado + warp: warps só com grab solto (`cursor_confined`).

### Confirmados e corrigidos (diagnóstico do usuário, procedente nos 2)

1. **Zona-morta no `O` — bug de port em `vse_boost_hold`, não VSE.** O ramo
   `|nx|<0.5`/centro/troca-de-lado retornava `(0,0)`, zerando o ALVO em meia
   tela (`nx=0.3` → VSE `0.225`, nós `0.0`); na troca de lado, um frame de
   zero; e com hold latched devolvia o alvo cru em vez de `lateral+hold`
   (ratchet vazava). Reescrito VSE-exato (`target = sign*(lateral+|hold|)`
   sempre) + testes (pass-through pequeno, persistência do boost, troca).
2. **Briga no A/D — gamepad parado via chase do stick.** `poll()` emite
   `Some(0.0)+stick` p/ qualquer pad conectado mesmo intocado (Steam
   Input/vJoy contam!); o chase (`app_events`) arrastava o volante a
   `1/1.2s` rumo a zero enquanto as teclas integravam p/ cima, e o gate
   `0.02` do merge alternava de vencedor a cada poucos ms. Fix: chase só
   com stick defletido (`vse_stick_chase`) — parado não escreve. Para
   confirmar no seu caso: rode com `OMSI_TRACE_STEER` e leia o `.tick.csv`;
   coluna `analog` preenchida sem `O` = dispositivo fantasma (desplugar
   isola em 1 comando).

---

## 14. Controle de pedais — auditoria VSE × openOMSI (demolir e reconstruir)

Cadeia VSE (`PedalInputController::{Update,Reset}`, `hpp:61-97`):
`W/S/TAB held + analog(-1=s/n)` → [analog≥0? direto : rampas] →
`throttle_raw/brake_raw/clutch_pedal_input` → física (origem desconhecida).
Acelerador: tip-in `0.16` na borda, `0→0.80/0.80s`, double-tap (≤0.40s)
kickdown `→1.0/0.16s`, queda `1/0.35s`. Freio: toque curto (<0.22s) latcha
degraus `10/25/50/80/100`, hold tangente `exp(-dt/τ)` com `τ 0.28→0.85`
(≤20→≥90 km/h), `W` cancela latched em `0.125s` (só sem `S`), emergência
`≥0.95`. Embreagem: mesma forma, `bite 0.20`, sobe `0.45s`, desce `0.18s`,
só com pedal (`HasClutchPedal`), `TAB` fora do mouse-drive. `dt` clamp 0.05.

Cadeia openOMSI (a demolir): `update()` — throttle `+2/s→0.85`, queda `1/s`;
freio `+1/s`, `pedal_hold`; embreagem instantânea/`0.7s`. Sem tip-in, sem
kickdown, sem degraus, sem tangente, sem cancelamento, sem bite.

Numpad (`8/4/6/5/2`): em AMBOS é luzes/views/`p.key` (câmbio OMSI) — nunca
steering. Nada a portar; registrado para não reinterpretar.

Mouse `O`: matemática idêntica (alvo/ratchet/slew/pedais `0.10`), verificado
linha a linha; `nx/ny` físicos dos dois lados. Mantido.

Reconstrução (só controle; física das rodas intacta): máquina de pedais VSE
em `KeyboardAxes` (estado prévio + timers + latch; emergência sem consumidor
= descartada), `amplify`→teto `1.0` e `pedal_hold=false`→queda
rápida preservados como opt-outs explícitos (não brigam: ramos, não soma).
`dt` clamp `0.05` como a referência (anti-hitch). Implementado e testado
(`pedals_as_vse_ramps_them`, `the_clutch_bites...`, neutro reescrito p/ `dt`
de frame).

---

## 13. Porte do steering da referência `2037585` (só steering, câmera intacta)

Referência: `openOMSI-recovered` @ `2037585 "Steering and mouse match the
source exactly"` (+ `5f91ba2`, `a33b9a0`, `60162c4`). Auditoria §12 acima.

### Portado

- `hold_steer`: campo + ramo no `update` (congela até mão no volante; qualquer
  tecla de direção/centering limpa) + handoff no `toggel` (`release_all` +
  sobras do mouse + `hold=true` ao sair; zera tudo ao entrar). Causa do
  "gira sozinho" pós-`O` (retorno puxava o volante entregue).
- `mouse_anchor`: neutro até o primeiro movimento a partir do cursor de
  ativação (helper `vse_anchor_neutral` + teste). Causa do salto/deriva ao ligar.
- Substeps fixos `0.01 s` (`(dt/0.01).round clamp 1..5`) na integração e no
  retorno do teclado (VSE `kFixedDt`), taxas recalculadas por substep.
- Slew linear no caminho vivo (`vse_mouse_slew`, já era a helper; o bloco
  usava `exp` inline).
- RMB: só clique puro (`rmb_moved < 4.0`) desliga `O`; arrastar (zoom) mantém.
  Campo `rmb_moved` acumulado no motion.

### Preservado do `main` (divergência consciente da referência)

- `mouse_owned` (teclado ignorado durante `O`; a referência soma via `max` —
  item 17 desta especificação exige VSE exato, sem soma).
- `keep_wheel` nos desligamentos por menu/RMB (continuidade; a referência só
  tem handoff no `toggle`).
- Setas nunca dirigem; sem ramos `steering_response`/`mouse_response`
  (defaults da referência == comportamento atual; sem nova superfície).
- `sens` default efetivo `0.7` (igual ao default da referência).
- Matemática `direct_*`, pedais, merge, FFB: idênticos, não tocados.
- Câmera (F1–F4, Space, zoom, orbit, `camera_arm`, `placing`), física,
  renderer, backend, Android/VR: zero bytes alterados nesta fase.

### Bateria diferencial (154 cenários × referência, fora do repo)

Harness temporário (`steer_diff`, repos intocados) com os dois `input.rs`
como módulos: silêncio, hold A/D, alternância rápida, W/S, soltar, em
8 velocidades × 6 `dt`s. Primeira rodada: 25 divergências, todas no retorno
com `dt` grande — o `pitch` era calculado uma vez por frame no `main` e por
substep na referência. Corrigido na origem (recomputar por substep):
**154/154 idênticos** (hold/alternância/pedais já eram `0.00e0`).

---

## 12. Recuperação do steering — auditoria `main` × referência `2037585`

Referência: `F:\Programação HD\openOMSI-recovered`, HEAD `2037585 "Steering and
mouse match the source exactly"`. Regra: só steering; câmera do `main` intacta.

### Verificado idêntico (não portar)

- `direct_steer`/`direct_ratchet`/`direct_pedals` == `vse_mouse_target`/
  `vse_boost_hold`/`vse_mouse_pedals`, linha por linha (só nomes diferem).
- Unwind Hermite + blend + taxas + retorno angular == `vse_*_inner`
  (fórmulas e constantes iguais; `sens` default efetivo `0.7` nos dois).
- Merge do `tick` idêntico (`match` analog/keys).
- Setas fora do drive no `main` (mais estrito que a referência; câmera —
  não reverter).

### Primeira divergência: handoff sem `hold_steer` (commits `60162c4`, `5f91ba2`)

A referência congela o volante ao sair de `O` (`hold_steer=true`, ramo no
`update` que segura até mão no volante); o `main` usa `keep_wheel` + retorno
VSE, que puxa o volante sozinho rolando. Causa do "gira sozinho" pós-`O`.

### Segunda divergência: sem `mouse_anchor` (commit `a33b9a0`)

A referência zera target/pedais até o mouse sair do ponto de ativação; o
`main` projeta o cursor parado (onde quer que esteja) como comando —
primeiro frame/teleporte e deriva ao ligar.

### Terceira divergência: sem substeps de `0.01 s` (commit `2037585`)

A referência integra teclado/retorno em `n=(dt/0.01).round clamp 1..5`
(VSE `kFixedDt`); o `main` integra com o `dt` do frame de uma vez.

### Quarta divergência: lag exponencial no caminho vivo

`vse_mouse_slew` (linear, fiel ao VSE `min(1,dt/tau)`) existe mas o bloco do
mouse usa `exp(-dt/tau)` inline. Trocar pela helper.

### Quinta divergência: RMB-off sem `click × drag` (referência: `rmb_moved`)

A referência só desliga `O` em clique puro (`moved<4.0`); arrastar com RMB
(zoom) mantém `O`. O `main` desliga em qualquer press. Campo `rmb_moved`
inexistente no `main`.

### Explicitamente NÃO portado (com motivo)

- `steering_response`/`mouse_response`/`camera_response` + telas: superfície
  nova de opções; defaults da referência (`heavy`/`direct`) == comportamento
  atual do `main`. Sem evidência de sintoma → sem port.
- Ramo OMSI-legado do mouse (`mouse_steering` com divisor de velocidade):
  compat; sintomas não o implicam.
- `wheel_force` align cap (FFB de saída, não cadeia de input).
- `fallback` com setas (câmera; `main` mais estrito, não reverter).
- `tick` merge, `direct_*`, pedais, `sens` default: idênticos.


### Testes

- `cargo fmt --check`: só diff pré-existente `crates/omsi-app/build.rs` (intocado).
- `cargo check --workspace`: ok. `cargo check --tests --workspace`: ok.
- `cargo test -p omsi-sim`: ok — 174 passed, 0 failed.
- `cargo test -p omsi-app --lib` (com `CARGO_INCREMENTAL=0`, linker incremental
  MSVC pré-existente): ok — 152 passed, 0 failed (147 + 2 roteamento `vse` +
  3 matriz teclado; `mouse_tests` OMSI preservados).
- `cargo test --workspace`: exit 0, nenhum `FAILED` / `could not compile`.
- `cargo build --release`: ok.
- `OMSI_ROOT`/compat: executado em 2026-10-01 contra
  `F:\SteamLibrary\steamapps\common\OMSI 2` — `omsi-check` exit 0
  (1321/1321 maps, 21296/21297 textures; 2 notas de conteúdo pré-existentes:
  demo-OTP sem tickets, `boot3.dds` DXGI 98) e `cargo test --workspace` com
  `OMSI_ROOT` exit 0, nenhum `FAILED`.
- Nenhum teste removido/desativado; warnings só `dead_code` de helpers de
  especificação (padrão já usado em `vse.rs`) e pré-existentes.

---

## 7. Validação runtime/comportamental (pós-implementação, sem mudar arquitetura)

Método: extrações puras 1:1 do dispatch (`vse_drag_action`, `vse_wheel_action`
em `vse.rs`, chamadas nos mesmos pontos de `app_events.rs`) + matriz
`fallback_action` × presets + auditoria `STOCK_KEYS` (`player.rs:
input_routing_tests`) — ou seja, "qual sistema consome este input neste contexto"
respondido por teste executado, não por leitura. Sem fixture de `App`/janela no
repo (nenhum teste constrói `App`), então frame-loops e `on_key` seguem cobertos
por leitura + checklist abaixo; nenhum comportamento foi alterado nesta etapa
(apenas extração + testes), exceto um caso documentado.

### 7.1 Checklist executado

| Caso | Resultado | Evidência |
|---|---|---|
| F1 `ArrowLeft/Right` → prev/next, sem steering | ✅ teste + leitura | `arrows_never_drive_in_any_preset` (6 presets × 4 setas); braços `on_key` F1/F2 |
| F1 `Space` → câmera `camera_std` | ✅ leitura | `view_reset_all_directions`: `cam_choice=(0,0)` ≡ `(std+0)%n` |
| F1 `MMB drag` → orbit | ✅ teste | `drag_routing_single_consumer_per_context` (Middle→Look nas 5 views) |
| F1 `RMB drag` → zoom (down=in) | ✅ teste+leitura | roteamento testado; ganho `dy/16.25` (=364 px→22.4 notches→×6.49, par VSE 6.5×) por leitura |
| F1 `Wheel` → nada | ✅ teste | `wheel_routing_only_f3_zooms` (driver/pax→Nothing com bus) |
| F2 `ArrowLeft/Right` → next instantâneo | ✅ teste+leitura | matriz setas + `CamBlend` só driver (F2 sem glide, como VSE) |
| F2 `Space/MMB/RMB/Wheel` | ✅ teste | mesmos roteamentos F1 (zoom RMB, wheel Nothing) |
| F3 `MMB drag` → orbit | ✅ teste | Middle→Look |
| F3 `Wheel` → zoom (`∓1.5/notch`) | ✅ teste | `Orbit`; `Ctrl+wheel`→Telephoto |
| F3 setas → orbit hold | ✅ leitura | bloco `outside` no frame (`look` wrap 360, pitch VSE) |
| F3 `Space` → look/orbit reset | ✅ leitura | `view_reset_all_directions` limpa `look`, `orbit=DEFAULT` |
| F4 `MMB drag` → orbit/turn | ✅ teste | Middle→Look |
| F4 `Alt+MMB` → pick mesh→ground→Z=0 | ✅ leitura | braço dedicado no handler MMB (mesh `surface_hit/body_hit` primeiro) |
| F4 `LMB` → retarget ground→Z=0 | ✅ leitura | `on_left` free + `vse_ground_hit` 14 bisseções + fallback |
| F4 `Wheel` → dolly (não FOV) | ✅ teste | free→Dolly |
| DRIVE `W/S/A/D` | ✅ teste | `wasd_drives_only_where_it_should` (3 presets + desconhecido; `omsi/arrows` opt-out D7) |
| Setas NÃO dirigem (vazamento) | ✅ teste | matriz 6×4 zerada + auditoria stock (setas só `view_interiorcam_∓`, com guarda `plain_arrow`) |
| `LShift/LCtrl` → gear↑/↓ só automático | ✅ leitura | braços com `is_manual_gearbox()` + gates de contexto; `shift_gear` condicional pré-existente |
| `N/R/1-7` condicionais | ✅ leitura | `Player::key:708-736` resolve via `bindings` do bus (automático ignora `kw_s_*` e vice-versa) |
| Combos `Alt+MMB`, `MMB/RMB+move`, `LMB+move` | ✅ teste+leitura | roteamento testado; drag de switch (`dragging/drag_delta`) intocado |
| Transições F1↔F2↔F3↔F4, `Space` por view | ✅ leitura | `look_key`/`sync_view_look` por view+câmera; reset por view |
| Sem stuck | ✅ leitura | focus-loss limpa `keys` + `mouse_look_btn` + `axes` |

### 7.2 Divergências reais encontradas nesta etapa

1. **Refactor do wheel quase mudou `driver/pax` sem bus** (dolly → no-op). Capturado
   na revisão do próprio refactor e corrigido na tabela (`has_player=false` →
   `Dolly`, teste cobre). Prova de que extração+teste pega o que leitura não pega.
2. **Escala do drag do mouse difere do VSE em valor absoluto**: o chamador entrega
   graus (`×0.15°/px`, `app_events.rs`/`look_by`), enquanto o VSE multiplica pixels
   por `0.245°/px` (F1) e `0.35°/px` (F3). Clamps/curvas têm paridade; o ganho
   absoluto, não. Item para comparação ao vivo VSE↔openOMSI lado a lado —
   **não alterado aqui** (etapa de validação, sem nova arquitetura).
3. **Curva de zoom**: `view_zoom` é multiplicador (`min 0.154 ≈ 1/6.5`) vs VSE
   `base/(1+5.5z)`. Alcance final equivalente (6.5×), forma da curva aproximada.
   Mesmo status de (2): comparar ao vivo, não alterar aqui.
4. **VSE `Up/Down` em F1/F2 = nada; aqui = look ±35°**: divergência consciente
   (item 8 da auditoria anterior já marcava `⛔ comentário-vs-código` no próprio
   VSE). Mantido o look (regressão OMSI seria pior); `Left/Right` seguem VSE.

### 7.3 O que esta etapa NÃO cobre (exige sessão ao vivo com VSE)

Sensação de ganho em px/s reais, FOV ao vivo, temporização do ease em wall-clock
com quantização de frame, e qualquer comparação pixel-a-pixel — sem `OMSI_ROOT`/
display nesta máquina. Nenhum item do checklist ficou sem ao menos um dos dois
níveis (teste executado ou leitura com `arquivo:linha`).


