# VSE → openOMSI — Auditoria Arquitetural de Portabilidade: Câmera + Volante + Dirigibilidade

> **REGRA FUNDAMENTAL DESTA FASE: NENHUMA IMPLEMENTAÇÃO.**
> Este documento é auditoria, não plano genérico. Não foi criado código Rust, stub, abstração, nem alterado input/câmera/física/renderer em nenhum dos dois repos.
> Origem (referência): `D:\Programação\Engine Simulador` — C++17.
> Destino: `F:\Programação HD\openOMSI` — Rust (`rust-version = "1.85"`, edition 2021).
> Data: 2026-09-30.
> Convenção neste doc: `arquivo:linha` = evidência. `⛔ EVIDÊNCIA INSUFICIENTE` = não localizado, não inventar. `⛔ NÃO EXISTE EQUIVALENTE DIRETO` = gap real no openOMSI.

---

## 1. Escopo

### 1.1 Incluído

- Feature 1 — Sistema completo de câmera VSE: F1 cockpit/motorista, F2 passageiro, F3 externa orbital follow, F4 externa livre no mundo, posições `.bus`, navegação/wrap, easing, movimento cabeça, reset `Space` (para `camera_std`, não índice 0), rotação/zoom/FOV/mouse/scroll/MMB/RMB/orbit/alvo/smoothing/clamp/estado/update por frame/interação câmera-veículo-renderer-input-colisão-mundo.
- Feature 2 — Volante completo: input, estado, velocidade, limites, centro/retorno, steering angle, relação volante-rodas-dynamics, física, timestep, filtros/clamp/deadzone/ganho, dependências 3D/câmera.
- Feature 3 — Dirigibilidade completa: volante+acelerador+freio, tecla `O` (mouse-drive no Play; luz interna no standalone), WSAD, estados/transições, física, torque/força, filtros/limites.
- Cadeia Renderer/Viewport/Raycast/Ground/Collision/Scene/Mesh/World/PreviewHost/VehicleDynamics.
- Mapeamento VSE → openOMSI com gaps 🟢/🟡/⛔ e step-by-step futuro (não executado).

### 1.2 Excluído

- Qualquer implementação Rust, stub, refatoração, bump de toolchain, troca de framework (`wgpu 29`, `winit 0.30`, `glam 0.30`), mudança desktop/mobile, alteração de `CONTRIBUTING.md`/`docs`.
- UI mobile, `touch.rs`, VR (`openxr.rs`), comandos específicos mobile.
- `steering-camera-v2` — explicitamente adiado até leitura desta auditoria.

### 1.3 Critério de aceite desta auditoria

Para cada comportamento deve ser possível responder: de onde nasce, quais sistemas participam, quais dados entram, como são transformados, para onde saem, qual cadeia equivalente no openOMSI. Se a resposta for “criar câmera parecida”, a auditoria está incompleta.

---

## 2. Estado atual do VSE

### 2.1 Onde mora a câmera (arquivos canônicos)

| Arquivo | Papel |
|---|---|
| `src/editor/SimulationPreviewHost.hpp:68-73` | `enum PreviewCameraMode{DriverCockpit=0,ExteriorChase=1,PassengerSaloon=2,FreeCam=3}`. Mapeamento F1=0,F2=2,F3=1,F4=3 confirmado em `SimulationPreviewHost.cpp:2684-2690` |
| `src/editor/SimulationPreviewHost.hpp:197-208,268-288,388-403,502-584` | API (`OnCameraPan,ResetCameraCenter,NotifySpaceKey,SwitchDriverCamera,SwitchPaxCamera,OnInteriorOrbit,OnInteriorZoomDrag,SetCameraMode,InitFreeCamFromChase,OnChaseOrbit,OnZoom`) + estado (`m_chaseYaw/Pitch/Distance`, `m_lookYaw/Pitch`, `m_zoomAmount`, `m_cockpit*/m_pax*`, `m_driverCameras/m_paxCameras`, transição, `m_prevKey*`) |
| `src/editor/SimulationPreviewHost.cpp:1254-1631` | `BindBusCameras, ApplyDriver/PaxInstant, Begin/AdvanceDriverEase, SwitchDriver/Pax, ResetCameraCenter, OnChaseZoom, OnZoom, OnInteriorOrbit/ZoomDrag, SetCameraMode, InitFreeCamFromChase` |
| `src/editor/SimulationPreviewHost.cpp:1799-2245` | `Update(dt,camera)` — física + polling setas/space + `UpdateCameraPose` |
| `src/editor/SimulationPreviewHost.cpp:2314-2448,2466-2519` | `UpdateCameraPose` por modo + `GetNominalRideHeight, GetVehicleWorldMatrix` |
| `src/editor/EditorCameraController.hpp:36-81,140-170` | Controlador orbital genérico (`OnOrbit/OnPan/OnZoom/OnPrecisionDolly/OnFlycamMove, SetOrbitTarget/Smooth, ScreenPointToRay, GetView/Projection`). Estado `m_target/m_targetDesired/m_distance=8.0/m_pitch=25/m_yaw=45/m_fov=60/near=0.05/far=2000/sens 0.35` |
| `src/editor/EditorCameraController.cpp:39-60,62-139,268-302,527-609` | `Update` interpola target `blend=1-exp(-10*dt)`, `OnOrbit/OnPan/OnZoom`, `SetOrbitTarget` vs `Smooth`, `ScreenPointToRay`, `UpdateCameraPositionFromOrbit,ClampAngles,SetLookAtDirect` |
| `src/editor/EditorViewportHost.hpp:86-99,387,484` + `EditorViewportHost.cpp:2238-2280,2652-3150,3395-3519,4685-4713` | Input/viewport: `OnL/M/RMouseDown,OnMouseMove/ Wheel,OnKeyDown/Up`. Ramo F4 LMB `:2263-2279`, Alt+MMB pick `:2691-2809`, Play keys F1-F4/Space `:3491-3518` |
| `src/editor/EditorWindow.cpp:187-203,290-305,394-425,511,529,569-598` | `WndProc → RemapClientToScene → OnL/M/RMouse, OnWheel, OnKeyDown` |
| `src/parsers/OmsiBusParser.hpp:24-35,66-69` + `OmsiBusParser.cpp:110-122,163-164,218-223,256-335,469-516` | `.bus` → `OmsiCameraDefinition{type,position[3],viewAngleOffset,fov,yaw,pitch,roll,tag,unitIndex}` → `driverCameras/paxCameras/reflection/standardCameraIndex/coupleBack` |
| `src/dynamics/MeshGroundEnvironment.hpp:81` + `MeshGroundEnvironment.cpp:621` | `SampleGround(x,y)` heightfield |
| `src/editor/PathTool.hpp:217` + `PathTool.cpp:985-1057` | `ScreenPointToTerrain(ray,MeshGround*,out)` marching + bisseção 14 iterações |
| `src/core/EngineRuntime.hpp:28-33` + `EngineRuntime.cpp:148-178` | `enum CameraMode{Cockpit,Orbit,Follow,FreeCam}` standalone hard-coded — **não é F1-F4 do editor, não lê `.bus`** |

### 2.2 F1/F2 — `.bus` → armazenamento → navegação → easing → reset

**Parse:**
`OmsiBusParser.cpp:256-291` `[add_camera_driver]` lê 7 floats `oX,oY,oZ,dist,fov,yaw,pitch`, `ConvertOmsiPointToVse` (`:110-122` identidade X=lat,Y=long,Z=up), correção `if(dist>0 && dist<2) dist=-dist` (`:274-276`, idem pax `:305-307`), `type/fov/yaw/pitch/roll=pitch`, tag `[view_schedule]/[view_ticketselling]` só driver (`:284-288`). `[add_camera_pax]` `:292-314` idêntico sem tag. `[set_camera_std]` `:218-223` → `standardCameraIndex`. `[couple_back]` `:331-335` → trail.

**Armazenamento Play:**
`SimulationPreviewHost.cpp:1254-1287` `BindBusCameras`: copia `driver/pax`, zera `unitIndex=0`, `AppendCoupleBackCameras` (`:1289-1323`, `unitIndex=1` p/ trail), zera look/zoom, `ApplyPaxCameraInstant(0)`, `idx=clamp(standardCameraIndex)`, `m_principalDriverCamIdx=idx`, `ApplyDriverCameraInstant(idx)`, força `DriverCockpit`.
`ApplyDriverCameraInstant` `:1325-1343` e `ApplyPaxCameraInstant` `:1345-1361` copiam `position→offset, AuthoredFov (fov>5?fov:60, :1242-1244), eyeDist=viewAngleOffset, yaw, pitch, unitIndex`, zeram looks/zoom, `m_isCamTransitioning=false`. Pax sem transição.

**Pose por frame:**
`UpdateCameraPose` `:2314-2396`: base `heading/pitch/roll` → eixos `rX/rY/rZ,fX/fY/fZ,uX/uY/uZ` (`:2328-2340`), `veh - rideHeight` (`:2342-2345`), ramo interior `offset + eyeDist*view`, `view=[sinY*cosP,cosY*cosP,sinP]` (`:2379-2381`), `targetLook=offset+25*view` (`:2386-2388`), `VehiclePointToWorld` (`:1443-1455`, `GetVehicleWorldMatrix` ou trailer se `unit>0`, Main subtrai ride) → `SetLookAtDirect`. `InteriorDisplayFov` `:1435-1441` `base/(1+5.5*z)` clamp `max(4,…)`, zoom cheio 6.5x.

**Navegação/wrap/easing:**
Polling `Update :2171-2207`: `Left→SwitchDriver(-1)`, `Right→+1` (edge `m_prevKeyLeft/Right`, sem repeat); Pax idem `SwitchPax∓1`; FreeCam `OnPan∓10px`, else `OnCameraPan∓1`.
`SwitchDriverCamera` `:1566-1589`: `WrapCameraIndex` (`:1246-1252` `<0→count-1, >=count→0`), early-out se igual, cross-unit (`unitIndex!=atual`) → `ApplyInstant`, senão `BeginDriverEaseTo`.
`BeginDriverEaseTo` `:1363-1400`: salva start pos/fov/yaw/pitch/zoom/eyeDist, alvo com yaw shortest `remainder(…,360)` (`:1377`), colapsa `visualYaw=m_cockpitYaw+m_look`, `duration=0.54s` (`:1398`, era 0.45), `m_isCamTransitioning=true`.
`AdvanceDriverEase` `:1402-1433`: `t=clamp(time/duration)`, `s=1-(1-t)³` ease-out (`:1407`), lerp todos, ao `t>=1` sela + `false`. Chamado em `Update :2222` e pausado `:1807`.
`SwitchPaxCamera` `:1591-1599`: sempre `ApplyInstant` — **sem easing**.
Smoothing corpo (não câmera direta) `AdvanceBodyVisualSmooth` `:2280-2312`: Z física direta, pitch `tau=0.10`, roll `tau=0.12,gain=0.78`, `a=1-exp(-dt/tau)`.

**Space:**
`NotifySpaceKey` `:1601-1606` edge `!m_prevKeySpace`. Fontes: `OnKeyDown VK_SPACE` (`EditorViewportHost.cpp:3491-3494`) + polling `GetAsyncKeyState` (`SimulationPreviewHost.cpp:2167-2169`) + pausado `:1806`.
`ResetCameraCenter` `:1608-1631`: `chaseYaw=0,pitch=12`, se `.bus` → `home=clamp(m_principalDriverCamIdx)` (**`[set_camera_std]`, não 0**), `mode=DriverCockpit`, se já driver `BeginEase(home)` senão `ApplyInstant(home)`; fallback sem `.bus` zera looks/zoom/offsets. `Back/Home → ResetCameraCenter` (`EditorViewportHost.cpp:3496-3498`).

**F1 vs F2:**
Modo `DriverCockpit` vs `PassengerSaloon` (`EditorViewportHost.cpp:3500-3506`). F1 com ease 0.54s + cross-unit instant; F2 sempre instant. Olhar: F1 `OnInteriorOrbit` (`:1531-1542`, clamp yaw ±90 pitch -35..+35, sens `0.35*0.70`, `zoomStab=1-0.25*zoom`) + `OnCameraPan→OnInteriorOrbit` (`:1558-1564`) + MMB drag (`EditorViewportHost.cpp:2919-2920`); F2 `OnInteriorOrbit` retorna early se `mode!=DriverCockpit` (`:1532`), MMB no-op. Zoom RMB ambos via `OnInteriorZoomDrag` (`:1544-1556`, `dy*0.70/364` clamp 0..1, `:2921-2922`). Wheel em interior no-op (`:3137-3140`). `m_saloonYaw/Pitch`, `m_cockpitPitchOffsetDeg` zerados nunca escritos fora Apply/Reset — `⛔ EVIDÊNCIA INSUFICIENTE` para tilt por setas Up/Down em interior apesar de comentário `:2159-2161`. Picking filtra viewpoint (`EditorViewportHost.cpp:1871-1874,1907-1908`).

### 2.3 F3 orbital follow

Init `Start :439-446`: `dist=wb*1.5+4.0` (`wb>1?wheelbase:6.08`), `yaw=0,pitch=12`, `ZFiltValid=false,visBodyValid=false`. `BindBusCameras` força `DriverCockpit` se `.bus`, senão `ExteriorChase :452`.
`UpdateCameraPose ExteriorChase :2398-2443`: `FOV=60`, `offset=[dist*sinCY*cosCP,-dist*cosCY*cosCP,1.6+dist*sinCP]`, pivotZ filtrado `tau=0.30` (`:2416-2424`), base yaw-only, `camW=veh+yr*offX+yf*offY, camZ=pivotZ+offZ`, `tgt=(vehX,vehY,pivotZ+1.4)`, `SetLookAtDirect`.
Gestos: `OnChaseOrbit(dx,dy)` inline (`SimulationPreviewHost.hpp:277-282`) `yaw-=dx*0.35, pitch+=dy*0.35 clamp -10..75`; `OnZoom(delta)` (`:1520-1529`) `dist-=delta*1.5 clamp 4..40`, ignora interior; `OnCameraPan` chase `OnChaseOrbit(dx*4,dy*4)` (`:1560`); `OnMouseMove` chase MMB||RMB → `OnChaseOrbit` (`EditorViewportHost.cpp:2924-2925`). Smoothing só pivotZ. `⛔ Sem tangente/curva além desse low-pass Z`.

### 2.4 F4 — cadeia mouse→alvo (dois caminhos distintos, não confundir)

Entrada: `VK_F4` (`EditorViewportHost.cpp:3512-3518`) → `SetCameraMode(FreeCam)` (`:1457-1472`, zera looks/zoom, invalida ZFilt se chase) + `InitFreeCamFromChase` (`:1474-1518`, copia matemática F3 yaw-only, `FOV 60`, `SetLookAtDirect+SetOrbitTarget` p/ sincronizar dist/yaw/pitch). Todo F4 respawna da pose F3, nunca resume pose livre velha. Depois detach: `UpdateCameraPose FreeCam no-op` (`:2445-2448`), `Update` só `camera.Update(dt)` p/ POI glide (`:2243-2245`).

**Caminho F4 oficial (LMB simples, sem Shift/Alt) — `OnLMouseDown :2259-2280`:**
`mouse(x,y)` já remapeado (`EditorWindow :300-304,423-425` → `OnLMouseDown(x,y,alt,shift):2238-2244`) → guarda `mode==FreeCam && !shift && !alt :2263` → `ray=ScreenPointToRay(x,y)` (`EditorCameraController.cpp:527-568`, usa `m_position/m_target/fov/viewport` vivos) → `hitOk=ScreenPointToTerrain(ray,m_roadGroundEnv.get(),hit)` (`PathTool.cpp:985-1057`: normaliza dir, bounds, passo `clamp(0.75/horiz,0.05,2.0)`, marcha `t=0.05..4000`, `SampleGround` (`MeshGroundEnvironment.cpp:621`), cruzamento + bisseção 14 iterações; heightfield, **não mesh exata, não veículo**) → fallback `Z=0` se `!hitOk && |dirZ|>1e-5` (`:2268-2272`) → `SetOrbitTargetSmooth(hit)` (`EditorCameraController.cpp:296-302`, `m_targetDesired=hit`, interpolado em `Update :39-55 blend=1-exp(-10dt)`) → `return`, nunca segue veículo.

**Caminho Alt+MMB genérico (editor, vale em Play se Alt) — `OnMMouseDown :2678-2811`:**
`m_isAltDown=alt||VK_MENU :2687` → se Alt: `ray=ScreenPointToRay :2692` → 1. mesh (`m_activeMeshBuffer` + `GetVehicleWorldMatrix-ride` ou identidade, ray→local, loop tris Möller-Trumbore `:2698-2771`) → 2. bounding sphere (`m_hasFocusBounds`, ray-esfera `:2775-2790`) → 3. plano Z=0 (`:2796-2802`) → `SetOrbitTargetSmooth :2807`. Prompt diz “Alt+scroll-click”: código separa **F4 LMB = terrain heightfield + Z=0 (sem mesh)** vs **Alt+MMB = mesh+esfera+Z=0**. `⛔ Nenhum Alt+scroll-click específico F4 além do genérico`.

**Gestos pós-entrada F4:** `OnMouseMove FreeCam :2901-2912` (`Ctrl+MMB→PrecisionDolly, Shift+MMB/RMB→Pan, Alt+RMB→PrecisionDolly, MMB||Alt+LMB||RMB→Orbit(dy,dx)`), `OnMouseWheel :3141-3144` FreeCam `m_camera.OnZoom` (dist orbital), setas FreeCam `OnPan∓10px` (`SimulationPreviewHost.cpp:2191-2214`). `PreviewHost` só modo+Init; `Ground` só no pick LMB; `Renderer` só lê câmera via `GetView/Projection`.

### 2.5 Volante + dirigibilidade (Play Mode é autoritativo; arcade/standalone são outros binários)

Estado canônico `src/core/VehicleState.hpp:489-493,197`: `steering_input [-1,1]`, `steering_angle_rad ~±0.698 (±40°)`, `steering_angle_deg ~±40°`, `steering_wheel_angle_deg -850..+850`, `steering_wheel_angle_rad`, `ffb_torque_nm [-15,+15]`.

**Tecla `O` — dois significados, não confundir:**
Play editor `O` = mouse-drive: `EditorViewportHost.cpp:3548-3556` borda sem repeat `SetMouseDriveEnabled(!)`, `keyup :4709-4711` limpa `m_prevKeyO`, `RMB :2666-2670` desliga, saída Play `:3332` desliga. Estado `SimulationPreviewHost.hpp:467-476` (`m_mouseDrive,NX,NY,SteerSm,BoostHold`). Ligar `:1097-1108` semeia `SteerSm=clamp(steering_input)` sem teleporte; `SetMouseDrivePos :1120-1123` clamp. Posição absoluta `EditorViewportHost.cpp:2837-2856` `nx=(x-w/2)/(w/2), ny=(h/2-y)/(h/2)`.
Standalone `src/main.cpp:1110-1143,1260-1264` `O` = luz interna, **não direção**.

**WSAD Play:** `keydown :3419-3434` `W→Throttle,S→Brake,A→SteerLeft,D→SteerRight`, `keyup :4688-4702` limpa + `E/H/N`. Flags `SimulationPreviewHost.hpp:463-466,493`. Polling por substep `kFixedDt=0.01` (`:1819-1843`, `clampedDt=min(dt,0.1)`, `while≥0.01 && <5`), `mouseDrive` ignora WASD físico (`:1828-1837`), `TAB` embreagem só se `HasClutchPedal` (`Drivetrain.cpp:2335-2341` `type=="manual"`). Pedais teclado→`held` (`:1846-1858`), mouse em `:2037-2040`. `DispatchDriverCommands :1781-1791` monta `SET_STEERING ±1.00` → `CommandProcessor.cpp:258-259` `clamp(v)` — caminho paralelo p/ dashboard/gamepad.

**Mouse→volante matemática:** `animation/CockpitDrag.hpp:102-112` `ax=|nx|,ay=|ny|, edge=0.75*ax, corner=lin(0.20,1,ax)*lin(0.10,1,ay), mag=edge+(1-edge)*corner, target=±mag` (sem deadzone direção). `SimulationPreviewHost.cpp:1129-1137` `thr=(ay<=0.10)?0:(ay-0.10)/0.90, brk=(dn<=0.10)?0:(dn-0.10)/0.60`. Ratchet quina `:2002-2026` (`lateral=0.75*|nx|`, excedente em `BoostHold`, limpa se centro/troca/OFF). Slew `:2028-2036` `tau=0.12s, slew=2.5/s (0.025/substep), lagged=sm+(target-sm)*min(1,0.01/0.12)`.

**Velocidade volante teclado `:1886-1965`:** `base=0.336*sens, min=0.145*sens` (`sens default 0.70 clamp 0.20-2.00, :254-257,500`), `speed_turn=0.38+0.62*exp(-0.045*v_kmh)`, `speed_unwind=0.48+0.52*exp(-0.032*v_kmh)`, `low_mult=2.0-1.0*smoothstep(20→45km/h)`, `steer_rate=clamp(base*low*turn,min,base*2.2)`, `unwind_base=clamp(base*low*unwind,min,base*2.4)`, pitch Hermite C1 por `|input|` (1.0→1.35→2.15→2.75→3.25), `v_ramp`, `blend`, `unsteer`. Integração `:1970-1986` `input±=rate*0.01`, unwind se contra. Standalone `main.cpp:1280` `STEER_RATE=2.0/s`.

**Centro/retorno só em movimento `:2049-2115`:** `v_ramp=clamp(v/5,0,1)`, `base=0.22*v_ramp*exp(-0.020*v)`, `half=cfg>100?cfg*0.5:850`, `angle=|input|*half`, fator ângulo `1.0/>360°, 0.20-1.0/45-360° smoothstep, 0.05-0.20/5-45°, max(0.015,a/100)/≤5°`, `auto=0.33+0.67*pitch`, `step=base*f*auto*0.01`, se `dist≤step||angle≤0.05` zera senão `∓step`; parado mantém posição. `main.cpp:1286-1317` idem.

**Limites:** `VehicleConfig.hpp:38-89,125-174` (`Axle.max_steering_deg`, `max_steering_degrees=1700`), `ChassisParser.cpp:117-121,168-172`, Play força `50.0:steering?` (`SimulationPreviewHost.cpp:780`), `ChassisMotion.cpp:441-446` default `50.0`, `m_maxWheelAngleRad`, `m_maxSteerDeg=cfg>100?cfg:1700 :402`, `halfRad` `:1036-1038` `wheel=input*half`.

**FFB duplo escritor, último vence:** prévio `SimulationPreviewHost.cpp:2043-2047` `T=-0.025*δ*v² clamp ±15`, definitivo `interface/ControlSystems.cpp:90-99` mesma fórmula (comentário hydraulic -96%). Ordem `:364-380` `ControlSystems→…→ChassisMotion` dentro de `m_manager->Update(0.01)` (`:2126`, `VehicleManager.cpp:48-49`) → `ControlSystems` sobrescreve todo tick.

**Damping/smoothing/deadzone — veredito:** `deadzone` direção `⛔ EVIDÊNCIA INSUFICIENTE` (só limiares `0.005 unwind`, `0.0001 retorno`, `1e-4 rad Ackermann`, `0.10 pedais mouse`; freio `kPedalDeadzone=0.01` `BrakePlant.hpp:50`). `damping` coluna `⛔ EVIDÊNCIA INSUFICIENTE` (só `denomYaw`, amortecedores suspensão). `smoothing` teclado `⛔` (integração direta); existentes: mouse `tau 0.12+slew`, yaw `lerp tau 0.10` (`ChassisMotion.cpp:1363-1364`), lateral blend, visual `OmsiTransformRig.cpp:203-220 alpha=1-exp(-rate*dt)`, `WheelVisualSpin.hpp:34-126 SmoothDamp 0.055s`.

**Pedais rampa temporal:** `interface/PedalInputController.hpp:1-98` + `.cpp:57-198` → `throttle_raw/brake_raw` → `ControlSystems.cpp:54-55,79-80` `throttle_position/brake_pedal` (curvas identidade `:37-49`). Acelerador: tip-in 0.16, rise cap/0.80s→0.80, kickdown double-tap ≤0.40s→1.0 em 0.16s, fall 1/0.35s. Freio: toque <0.22s→degraus 0.10/0.25/0.50/0.80/1.00 latch, hold `exp(-dt/τ)` τ 0.28@≤20→0.85@≥90, W cancela em 0.125s.

**Física longitudinal (ordem fixa `:364-380`):** `throttle_position→Drivetrain.cpp:586,1175… T_drive`, `brake_pedal→BrakePlant.cpp:62-119 F_service=pedal*g_max*m*g*air*low*stop_ease (g_max 0.385/0.504 emerg, low 1.00≤10→0.90@20→0.70@60→0.50≥90, stop_ease lerp 0.95→1 easeOutQuart v/5), F_total=max(service,parking 0.65*Fz)` → `Drivetrain:721 brake_cmd=max(brake_raw,brake_pedal)` → `VehicleDynamics:102,112-137,197-202 F_aero=0.5*1.225*Cd*A*v² (Cd 0.40 A 8-8.5), F_roll=Crr*m*g*cosθ (0.006), F_slope, a=F/m` → `Drivetrain:1779-1837` única escrita `speed_mps=clamp(v,-8,70)`.

**Volante→ângulo→rodas→yaw→pose (`ChassisMotion.cpp:773-1467` sobrescreve `VehicleDynamics.cpp:76-92` no mesmo tick):** `delta_cmd=steering_angle_rad :775`, `L0=|x_steer-x_rear|≥1.5 :810`, `dirSpeedFactor 1>25→0>40 :816-818 * ERA ramp :833-845`, `f_curva` tabela (`CurveAuthority.hpp:39-56` Rigid4x2 1.0,6x2 0.70,6x2_dir 0.95/0.70,ArtPusher 0.90,MDA 0.88/0.60,Puller 0.85), eixo 0 `u=clamp(delta/max) :910, inner=|u|*max :914, r_in=L0/tan(inner) :919, r_out=r_in+track :920, outer=atan(L0/r_out) :921`, direcional `δ_inv=ratio*δ*fade clamp ±max :981-985 (ratio -0.3 PlanarKinematics.hpp:60-67)`, `R_bicycle=L/(tanδ*f) :929-932`, `wheel=input*halfMax :1037-1038`, `ω=v/R, spin+=ω*dt`, hub `vx=v+r*ly, vy=u-r*lx :1189-1190`, roda `v_lat/v_long :1194-1195`, `κ=(ωR-v)/|v| clamp ±5 :1200-1211`, `α=atan2(v_lat,|v_long|) :1214-1220`, `Fx=μFz, Fy=-μFz blend -cαα (cα≥60000 ou Fz*4.5), elipse ≤1 :1264-1296`, `Fx_body/Fy_body (scrub 0.06) :1303-1305`, `Mz=Σ(lyFx-lxFy) :1311, Izz=m(L²+track²)/12 :1167, r_dot=Mz/Izz :1337`, `r_target=(v/L1)tanδf :1352-1353 clamp ±0.75g/|u| :1355-1356, r=lerp(r,r_target,dt/0.10) clamp ±3.0 :1363-1365`, `v_lat_target=-r*ly_rear :1357, blend kinWeight=(5-|u|)/4 :1382, clamp ±4.0`, `ay=v_dot+u r clamp ±0.65g :1399-1403`, `heading+=r dt :1451, pos+=v dt :1461-1462`. Articulado `StepWithTractor :2389-2395`, holdSpeed, anti-jackknife `v*=0.8`. `SteeringIcrPivot.hpp:9-13,34-76` **PROIBIDO alimentar yaw** (só cue visual).

**Visual:** `SimulationPreviewHost.hpp:321-323,336-339` getters, `animation/AnimationDriver.cpp:179-182,243,302-378` captura + `axle_steering_N_L/R→wheelSteerRad, steering_3, cp_lenkrad→lastSwAngleDeg, wheel_rotation→-physical*spinScale`, `OmsiTransformRig.cpp:198-233` clamp+delay+slew→matriz. `GetVehicleWorldMatrix :2482-2508` de heading/pitch/roll+pos-ride → malha; câmera segue.

---

## 3. Estado atual do openOMSI

### 3.1 Toolchain, CONTRIBUTING, docs

- `Cargo.toml:32` `rust-version="1.85"`, edition 2021, resolver 2, version 0.1.0. `VERSION:1` `0.1`. `docs/BUILDING.md:13` stable 1.85+, `README.md:149` idem. CI `release.yml:49,110,143,189` `dtolnay/rust-toolchain@stable` sem pin. Glob `rust-toolchain*` não encontrado — toolchain “stable atual ≥1.85”, não pinned. Perfis `Cargo.toml:54-68` release/dev/android. Deps: `wgpu 29, winit 0.30, glam 0.30, rayon 1.10, parking_lot 0.12, hashbrown 0.15`.
- `CONTRIBUTING.md:1-22`: no original code/assets, `OMSI_ROOT` + skip sem instalação, compatibility first, `cargo run -p omsi-check` antes/depois de mudanças maiores, one change per PR, `cargo test --workspace` + `cargo build --release`, `rustfmt`, `README#repository-layout` + `ARCHITECTURE.md` + `FORMATS.md`.
- Docs lidos: `ARCHITECTURE.md` (tabela Delphi→crate, threading, memory, frames, roadmap 1-25), `USER_GUIDE.md:55-116` (W/S/A/D, Shift+D, `O` mouse steering, F1-F4, F5-F8, Space, Home, WASD+QE free, orbit wheel), `ANDROID.md` (mobile mesmo renderer/sim, `android.rs, launcher/mobile.rs, touch.rs, OMSI_TOUCH/MOBILE/INPUT`), `VR.md` (só Windows OpenXR DX12, `openxr.rs #[cfg(windows)]`). Não há `docs/input.md/camera.md/renderer.md/fisica.md` dedicados.

### 3.2 Estrutura `crates/` (19 + `tools/omsi-check`, `Cargo.toml:3-25`)

`omsi-cfg, omsi-script, omsi-o3d, omsi-model, omsi-scenery, omsi-map, omsi-vehicle, omsi-timetable, omsi-content, omsi-texture, omsi-geometry, omsi-render, omsi-sim, omsi-audio, omsi-net, omsi-plugin(+/demo), omsi-ui, omsi-launcher-core, omsi-app`.

| Sistema | Local real |
|---|---|
| input | `omsi-sim/src/input.rs` (`EngineAction, KeyboardAxes`); `omsi-app/src/input_script.rs` (winit→ações, `OMSI_INPUT`); `omsi-app/src/controllers.rs+dinput.rs #[cfg(windows)]+mac_hid.rs+evdev_ff.rs+platform.rs:tilt_steering()`; `omsi-app/src/touch.rs`; `omsi-content` (`keyboard.cfg, gamectrler.cfg`) |
| câmera | `omsi-vehicle/src/vehicle.rs` (def `.bus`); `omsi-app/src/player.rs:1611-1757` (`camera(), camera_clipped(), camera_look()`); `omsi-app/src/camera_util.rs` (`default_camera, cursor_ray, ORBIT_*, mirror_view`); `omsi-app/src/camera_arm.rs` (`SpringArm, free_length`); `omsi-render/src/lib.rs` (`Camera, Renderer, Scene`) |
| renderer/world/scene | `omsi-render/src/lib.rs` + `shader/enhanced/post/sky*.wgsl, clouds.rs, atmosphere.rs`; `omsi-app/src/scene.rs` (`World, TileState, VehicleRender, ObjectType; :2316 ground_height, :2359 camera_ground, :2412 walk_height, :2453 walk_height_near`); `omsi-app/src/lib.rs:110` usa `omsi_render::{Camera,Renderer,Scene}` |
| vehicle/physics | `omsi-vehicle/src/vehicle.rs` (`Vehicle, Axle, Camera, Coupling`); `omsi-sim/src/vehicle.rs` (`VehicleType:107, VehicleInstance:809, update():1984, step_physics():1293, step_rigid():1534, update_ai():2052`); `omsi-sim/src/physics.rs` (`Controls:16, VehiclePhysics:36, step():78`); `omsi-sim/src/rigid.rs` (`RigidBody:382, step():584, collide():1123, GroundProbe:110, CoupledPart:363`) |
| collision/ground/raycast | `omsi-sim/src/collision.rs` (`Obb, Contact, CollisionWorld, MeshShape, MeshObstacle`); `omsi-geometry/src/lib.rs:1315 ray_triangle, :1373 ray_mesh_hit, :1414 ray_near_sphere, :1421 ray_mesh, :1467 ray_triangles`; `omsi-app/src/placing.rs:24 ground_hit()`; `player.rs:1265 pick(), :1271 html_hit(), :1341 hovered_part(), camera_util.rs:48 ray_may_hit()` |
| cockpit/gameplay | `VehicleInstance+omsi-model (mouse_event)`, `player.rs:pick()/nearest_hits()` via `ray_mesh`, `app_events.rs:586-677`, `input_script.rs`, `driver.rs` (`Wheel:71`), `schedule/bus_service/duty_start/money/humans/traffic/services/situation/career/menu/game_lists/describe` |

### 3.3 Equivalentes já existentes (resumo; detalhe §15)

- F1-F4: `input_script.rs:376-381` `F1→driver,F2→pax,F3→outside,F4→free`, `player.rs:1667 camera_look`, `app.rs:211,217-220` (`view,view_looks,view_zoom,look,orbit`), `app_events.rs:869,952,1277,1365+`, `offscreen.rs:966,1382,1648`. 🟢
- `.bus` positions: `vehicle.rs:16 Camera{pos,dist,fov,yaw,pitch,extra}`, `:101-107` (`cameras_driver/pax/reflexion, view_schedule/ticketselling, camera_std, camera_outside_center`), `:144 read_camera()`, `:315-322` parser, `player.rs:1679-1706,1736-1742`, `camera_util.rs:298-314`. Coordenadas `.cfg` x right/y fwd/z up, z=0 pneus (`ARCHITECTURE.md:132-134`). 🟢 (sem glide automático exceto `driverview_smooth` em `game_lists.rs:122, launcher/pages.rs:667`).
- Orbit: `camera_util.rs:76-80 ORBIT_DEFAULT 10.0/MIN 3.5/MAX 40.0`, `:98 offscreen_orbit 18m`, `app.rs:220 orbit`, `lib.rs:524` init, `player.rs:1734-1754` construção (centro `outside_center` + `body_rotation`, yaw `heading-35+look`, pitch `-15+look`), `player.rs:1619-1657 camera_clipped→camera_arm::free_length+SpringArm::update`, `camera_arm.rs:398 free_length` (5 raios vs `Blocker+camera_ground`, MARGIN 0.4, CLEARANCE 0.6), `:527 SpringArm` (HOLD 0.35s,EASE 3.0,PULL_IN 18.0,ARM_MIN 1.0), `app_events.rs:1378-1388,2376` wheel, `input_script.rs:1164-1165 orbit`, testes `:583-722`. 🟢
- Raycast→ground→target: fragmentos `cursor_ray (camera_util.rs:112 via Camera::ray)`, `ray_mesh/ray_triangle (omsi-geometry)`, `placing.rs:24 ground_hit` (marcha `t+=max(t*0.01,0.25)` + bisseção 24 sobre `ground_height`, usado em `placing_frame:80-81`), `scene.rs ground_height/camera_ground/walk_height`, `player.rs pick/nearest_hits` (só bus), `camera_arm free_length` (5 raios, só câmera externa). 🟡 parcial. `⛔ NÃO EXISTE serviço genérico raycast→GroundHit|TargetHit único; NÃO EXISTE raycast contra CollisionWorld/Obb p/ target mundo (editor pick é proximidade ao centro, Tab próximo); NÃO EXISTE GetHeightAbovePoint genérico fora probe scripts (vehicle.rs:2032-2046)`.
- Steering state: distribuído, sem struct única — `KeyboardAxes{steering,…} input.rs:35, update(dt):102 (taxa 0.8/(1+v/45), mola, linear 0.05, snap <0.0003)`, `Controls{throttle,brake,clutch,steering} physics.rs:16`, `VehiclePhysics{steer_deg,max_steer_deg} :36, step():78 (target=steering*max, rate max*2.5*dt)`, `rigid.rs:584 step + Axle_Steering_*`, `player.rs:2015 mouse_steering((2x/w-1)/max(1,kmh/10)*inv_min_turnradius)`, `app.rs:133 mouse_steer`, `app_events.rs:630-677`, `driver.rs:71 Wheel + find_wheel/steer_hands (REST ±70°, RANGE)`. 🟢 distribuído.
- Drive `O`: `input_script.rs:237-241 KeyO→toggel_mouse_ctrl`, `USER_GUIDE.md:80-88`, `app.rs:130 mouse_drive, :133 mouse_steer`, `lib.rs:484-485` init false, `game_lists.rs:125,326-334` toggle, `input_script.rs:2221-2232`, `app_events.rs:136-148,619-677,2277,2062`, `player.rs:2015-2067` + testes. 🟢
- Desktop/mobile: `platform.rs:11 MOBILE=cfg!(android), :33 touch_controls()=MOBILE||OMSI_TOUCH, :17 exit()`, `android.rs`, `launcher/mobile.rs`, `touch.rs` (`Player::analog`, Material Symbols via `omsi-ui`), `Cargo omsi-app:64-94` (`arboard,rfd` not-android; `windows,openxr,dx12` windows; `objc2,hid` macos; `android_logger,jni` android), `cfg` em `openxr/dinput/mac_hid/evdev_ff/android`. Regra: desktop-only em `omsi-app` sob `cfg(not(android))/cfg(windows)` ou `platform/android/touch/launcher/mobile`, nunca em `sim/vehicle/render/geometry/map`.

---

## 4. Mapa de arquivos VSE → openOMSI

| VSE | openOMSI equivalente | Nota |
|---|---|---|
| `src/editor/SimulationPreviewHost.hpp/.cpp` (modo, Bind/Apply/Ease/Switch/Space, Update, UpdateCameraPose, mouse-drive, steer integrate) | `crates/omsi-app/src/player.rs` + `app.rs` + `app_events.rs` + `input_script.rs` | VSE monolito editor; openOMSI separado Player/App/Events/Script. 🟡 |
| `src/editor/EditorCameraController.hpp/.cpp` (orbit genérico, ScreenPointToRay, SetOrbitTarget) | `crates/omsi-app/src/camera_util.rs` + `camera_arm.rs` + `crates/omsi-render/src/lib.rs (Camera)` | 🟡 — orbit existe, `ScreenPointToRay→cursor_ray+Camera::ray`, `SetOrbitTarget→orbit+SpringArm` parcial |
| `src/editor/EditorViewportHost.hpp/.cpp` + `EditorWindow.cpp` (Win32→Host, F1-F4/Space, mouse dispatch, F4 pick) | `crates/omsi-app/src/input_script.rs` + `app_events.rs` + `placing.rs` | Win32→winit. 🟡 |
| `src/parsers/OmsiBusParser.hpp/.cpp` (câmeras `.bus`) | `crates/omsi-vehicle/src/vehicle.rs` (`read_camera`, `Vehicle::cameras_*`) | 🟢 direto, checar `dist` correção VSE vs OMSI |
| `src/dynamics/MeshGroundEnvironment.hpp/.cpp` + `src/editor/PathTool.hpp/.cpp` (SampleGround, ScreenPointToTerrain) | `crates/omsi-app/src/scene.rs (ground_height/camera_ground/walk_height)` + `placing.rs:ground_hit` + `crates/omsi-geometry/src/lib.rs (ray_*)` | 🟡 — sem serviço unificado |
| `src/core/VehicleState.hpp` (steering_input/angle/wheel/ffB) | `crates/omsi-sim/src/input.rs (KeyboardAxes)` + `physics.rs (Controls,VehiclePhysics)` + `rigid.rs` | 🟡 distribuído, sem struct única |
| `src/interface/PedalInputController.hpp/.cpp` + `ControlSystems.hpp/.cpp` (pedais, FFB definitivo) | `crates/omsi-sim/src/input.rs + physics.rs` | 🟡 — sem tip-in/kickdown/degraus latch OMSI-like |
| `src/dynamics/VehicleDynamics.cpp` + `ChassisMotion.cpp` + `PlanarKinematics.hpp/CurveAuthority.hpp/SteeringIcrPivot.hpp/WheelTirePlant.hpp` | `crates/omsi-sim/src/vehicle.rs (step_physics)` + `physics.rs` + `rigid.rs` + `ai_motion.rs` | 🟡 — openOMSI física simples/rígida, sem Ackermann por roda + `f_curva` + direcional + elipse |
| `src/systems/powertrain/Drivetrain.cpp` + `src/systems/auxiliary/BrakePlant.hpp/.cpp` | `crates/omsi-sim/src/vehicle.rs + physics.rs` | 🟡 |
| `src/animation/AnimationDriver.cpp` + `OmsiTransformRig.cpp` + `CockpitDrag.hpp` | `crates/omsi-app/src/driver.rs` + `crates/omsi-sim/src/anim.rs` | 🟡 |
| `src/api/CommandProcessor.cpp` (`SET_STEERING`) | `OMSI_INPUT script` + `EngineAction` | 🟡 |
| `src/core/EngineRuntime.cpp` + `src/simulation/VehiclePhysicsSystem.hpp` (arcade) | `crates/omsi-sim/src/physics.rs + rigid.rs` | Não é Play Mode; ignorar p/ paridade |

---

## 5. Mapa de classes/structs VSE → Rust

| VSE | Rust | Gap |
|---|---|---|
| `PreviewCameraMode` (`SimulationPreviewHost.hpp:68`) | `view: String ("driver"/"pax"/"outside"/"free")` (`app.rs:211`, `input_script.rs:376`) | 🟡 — string vs enum; mapear |
| `SimulationPreviewHost` estado câmera (`:502-584`) | `Player + App{view,view_looks,view_zoom,look,orbit,mouse_drive,mouse_steer}` (`player.rs, app.rs`) | 🟡 |
| `EditorCameraController` (`EditorCameraController.hpp:36`) | `omsi_render::Camera` + `camera_util::{default_camera,cursor_ray}` + `camera_arm::{SpringArm}` | 🟡 |
| `OmsiCameraDefinition` + `OmsiBusDefinition{driverCameras,paxCameras,reflection,standardCameraIndex}` (`OmsiBusParser.hpp:24,66`) | `omsi_vehicle::vehicle::Camera{pos,dist,fov,yaw,pitch,extra}` + `Vehicle{cameras_driver/pax/reflexion,view_schedule/ticketselling,camera_std,camera_outside_center}` (`vehicle.rs:16,101-107`) | 🟢 |
| `VehicleState{steering_input,steering_angle_rad/deg,wheel_angle_deg/rad,ffb}` (`VehicleState.hpp:489`) | `KeyboardAxes{steering,…} (input.rs:35)` + `Controls{steering} + VehiclePhysics{steer_deg,max_steer_deg} (physics.rs:16,36)` + `RigidBody Axle_Steering_*` | 🟡 |
| `PedalInputController` | `KeyboardAxes + Controls` | 🟡 — sem latch/degraus |
| `MeshGroundEnvironment` | `scene::World{ground_height,camera_ground,walk_height}` + `TileSurface` | 🟡 |
| `PathTool::ScreenPointToTerrain` | `placing::ground_hit` | 🟡 — 14 vs 24 iterações, heightfield vs texel |
| `CockpitDrag::MouseSteerTarget` | `player::mouse_steering` | 🟡 — fórmulas diferentes (ver §10) |

---

## 6. Mapa de funções VSE → Rust

| VSE | Rust | Gap |
|---|---|---|
| `BindBusCameras` (`:1254`) | `vehicle::read_camera + Vehicle load` (`vehicle.rs:144,315`) | 🟢 |
| `ApplyDriver/PaxInstant` (`:1325,1345`) | `player::camera_look driver/pax index` (`player.rs:1679-1706`) | 🟢 lógica, 🟡 sem `unitIndex` trail idêntico (openOMSI `pax_camera_count` inclui trailers `:1661`) |
| `WrapCameraIndex` (`:1246`) | `(camera_std+cam_choice)%n` (`player.rs:1679`) | 🟢 |
| `SwitchDriverCamera + Begin/AdvanceDriverEase` (`:1566,1363,1402`) | `⛔ NÃO EXISTE EQUIVALENTE DIRETO` p/ ease 0.54s `s=1-(1-t)³` + shortest yaw + cross-unit instant (só `driverview_smooth` setting) | ⛔ |
| `SwitchPaxCamera` instant (`:1591`) | `cam_choice` (`app_events.rs:1277`) | 🟢 |
| `ResetCameraCenter home=principal` (`:1608`) | `Space/Home` (`input_script`, `app_events`) — verificar se volta p/ `camera_std` ou índice 0 | 🟡 — checar |
| `OnInteriorOrbit/ZoomDrag, InteriorDisplayFov` (`:1531,1544,1435`) | `view_looks/view_zoom/look` (`app.rs, app_events.rs:1365+`) | 🟡 — consts diferentes |
| `OnChaseOrbit/OnZoom` (`hpp:277, cpp:1520`) | `orbit` + `camera_clipped/SpringArm` (`player.rs:1619, camera_arm.rs`) | 🟢 conceito, 🟡 consts/clamps |
| `UpdateCameraPose Driver/Chase` (`:2314,2398`) | `camera_look + camera_clipped` (`player.rs:1611,1667`) | 🟡 — reimplementar matemática F3 exata se paridade |
| `SetCameraMode/InitFreeCamFromChase` (`:1457,1474`) | `view="free" + default_camera([mapcam])` (`camera_util.rs:6`) | 🟡 — F4 VSE respawna de F3; openOMSI free é mapcam |
| `ScreenPointToRay` (`EditorCameraController.cpp:527`) | `cursor_ray + Camera::ray` (`camera_util.rs:112`) | 🟢 |
| `ScreenPointToTerrain` (`PathTool.cpp:985`) | `ground_hit` (`placing.rs:24`) | 🟡 |
| `SetOrbitTarget/Smooth` (`:268,296`) | `orbit dist + SpringArm::update` (`camera_arm.rs:527`) | 🟡 — sem `targetDesired` genérico mundo |
| `MouseSteerTarget` (`CockpitDrag.hpp:102`) | `mouse_steering` (`player.rs:2015`) | 🟡 — fórmulas distintas |
| `ComputeAckermannSteering + UpdatePlanarCoupledDynamics` (`ChassisMotion.cpp:773,1160`) | `step_physics/step_rigid` (`vehicle.rs:1293, rigid.rs:584`) | ⛔ — sem Ackermann/f_curva/direcional/elipse |
| `FFB -kδv²` (`ControlSystems.cpp:90`) | `⛔ NÃO EXISTE EQUIVALENTE DIRETO` (FF nativo `controllers/dinput/evdev_ff`, rumble script) | ⛔ |

---

## 7. Mapa de estados — quem escreve, quem lê

| Estado | Escritor VSE | Leitor VSE | Equivalente openOMSI |
|---|---|---|---|
| `m_driverCameras/m_paxCameras/m_principalDriverCamIdx/m_currentDriver/PaxIdx/m_cockpitUnitIndex` | `BindBusCameras/Apply/Switch/BeginEase` | `UpdateCameraPose` | `Vehicle::cameras_*, camera_std + App::view/cam_choice` (leitor `camera_look`) |
| `m_cockpitOffset/Fov/Yaw/Pitch/EyeDist, m_saloonOffset…` | `Apply/Begin/AdvanceEase` | `UpdateCameraPose` | `Camera def + driver_eye()` |
| `m_lookYaw/Pitch, m_zoomAmount` | `OnInteriorOrbit/ZoomDrag` | `UpdateCameraPose→InteriorDisplayFov/SetFov` | `view_looks/view_zoom/look` |
| `m_chaseYaw/Pitch/Distance, m_chaseCamZFilt` | `Start/OnChaseOrbit/OnZoom/SetCameraMode/Reset` | `UpdateCameraPose Chase + InitFreeCam` | `orbit + SpringArm` |
| `m_cameraMode` | `SetCameraMode/Bind/Reset/F-keys` | `Update/UpdateCameraPose/OnMouseMove/Wheel` | `view string` |
| `m_isCamTransitioning + start/target ease` | `Begin/AdvanceEase/SetCameraMode` | `Update` | ⛔ sem estado |
| `steering_input` | `A/D integrate, mouse lag/slew, SET_STEERING, retorno` | `VehicleDynamics→ChassisMotion` | `KeyboardAxes::steering → Controls::steering` |
| `steering_angle_rad/wheel_angle` | `VehicleDynamics:81 + ChassisMotion:773` | `FFB, visual, Mz/yaw` | `VehiclePhysics::steer_deg + Axle_Steering_* + Wheel var` |
| `m_mouseDrive/NX/NY/SteerSm/BoostHold` | `SetMouseDriveEnabled/Pos, Update mouse` | `Update steer/pedais` | `App::mouse_drive/mouse_steer + analog` |
| `pedal_*_held/analog → throttle_raw/brake_raw → throttle_position/brake_pedal` | `W/S poll, mouse, PedalInputController, ControlSystems` | `Drivetrain/BrakePlant` | `KeyboardAxes→Controls::{throttle,brake}` |
| `speed_mps/heading/pos` | `Drivetrain (única escrita v), ChassisMotion (heading/pos)` | `câmera, renderer, ground probe` | `VehicleInstance::physics/rigid + World` |

---

## 8. Mapa de dependências diretas e indiretas

**F1/F2:** `.bus texto → OmsiBusParser::ConvertOmsiPointToVse/ParseFloat → BindBusCameras (+trail LoadFromFile) → Apply/Begin/AdvanceEase (AuthoredFov, Wrap, remainder yaw) → UpdateCameraPose (GetNominalRideHeight→ChassisMotion::GetStaticRideHeight senão R+0.30; GetVehicleWorldMatrix/Trailer + visBodySmooth tau 0.10/0.12; VehiclePointToWorld) → EditorCameraController::SetLookAtDirect/SetFov → GetView/Projection (LookAt LH + reverse-Z persp) → EditorViewportRenderer (SetCurrentCameraViewpoint, PickCockpitClickable filtra viewpoint) → imagem.**

**F3:** `Start(dist) → SetCameraMode(ExteriorChase) → UpdateCameraPose Chase (veh world_pos/heading, pivotZ tau 0.30, base yaw-only) → SetLookAtDirect → Render (SetCurrentCameraViewpoint(mode), matrizes mundo). Input MMB/RMB→OnChaseOrbit, wheel→OnZoom, setas→OnCameraPan(x4).`

**F4:** `F4→SetCameraMode(FreeCam)+InitFreeCamFromChase (copia F3) → detach → LMB: ScreenPointToRay (m_position/target/fov/viewport) → PathTool::ScreenPointToTerrain→MeshGroundEnvironment::SampleGround (RebuildRoadGroundFromTransform BuildFromMesh) → fallback Z=0 → SetOrbitTargetSmooth→Update blend 1-exp(-10dt)→UpdateCameraPositionFromOrbit → GetView/Projection → Render. Alt+MMB: mesh buffer (GetVehicleWorldMatrix-ride ou identidade, Möller-Trumbore) → esfera → Z=0 → Smooth.`

**Volante/condução:** `WASD/O/mouse/SET_STEERING → flags/NX,NY → steer integrate (sens, exp velocidade, low_mult, Hermite pitch, lag/slew mouse, ratchet BoostHold) + retorno (só em movimento, fator ângulo smoothstep, auto) → steering_input → VehicleDynamics linear (clamp) → ChassisMotion Ackermann (L0, dirFade, f_curva, inner/outer, direcional ratio -0.3, R_bicycle) → WheelTirePlant (κ,α,Fx,Fy,elipse,scrub 0.06) → Mz/Izz→r_target→r lerp tau 0.10→heading/pos + v_lat/ay clamps → Drivetrain/BrakePlant longitudinal (throttle/brake pedais com rampas) → speed_mps única escrita → GetVehicleWorldMatrix → AnimationDriver/OmsiTransformRig visual + câmera follow. FFB `-0.025δv²` prévio sobrescrito por ControlSystems no tick.`

---

## 9. Cadeias completas (causais, com arquivos)

1. **F1 inicial:** `.bus[add_camera_driver]` (`OmsiBusParser.cpp:256`) → `ConvertOmsiPointToVse` (`:268`) → `BindBusCameras` (`SimulationPreviewHost.cpp:1254`) → `ApplyDriverCameraInstant(std)` (`:1284`) → `UpdateCameraPose Driver` (`:2351`) → `SetLookAtDirect` (`EditorCameraController.cpp:597`) → `GetView/Projection` (`:391,469`) → `Render`.
2. **F1 ←/→:** `WM_KEYDOWN VK_LEFT` (`EditorWindow.cpp:598`) → `OnKeyDown return` (`EditorViewportHost.cpp:3520`) → `Update poll` (`SimulationPreviewHost.cpp:2177`) → `SwitchDriverCamera` (`:1566`) → `Wrap` (`:1246`) → `BeginEase`/`Apply cross-unit` → `AdvanceEase` (`:1402`) → `UpdateCameraPose`.
3. **F1 olhar:** `MMB drag` (`EditorViewportHost.cpp:2678`) → `OnMouseMove Driver` (`:2919`) → `OnInteriorOrbit` (`:1531`) → `m_look` → `UpdateCameraPose`.
4. **F1 zoom:** `RMB drag` → `OnMouseMove interior RMB` (`:2921`) → `OnInteriorZoomDrag` (`:1544`) → `m_zoomAmount` → `InteriorDisplayFov` (`:1435`) → `SetFov`.
5. **Space:** `VK_SPACE` (`:3491`) ou `Update spaceEdge` (`:2167`) → `NotifySpaceKey` (`:1601`) → `ResetCameraCenter home=principal` (`:1608`) → `Begin/Apply` → `pose`.
6. **F3:** `VK_F3` (`:3508`) → `SetCameraMode(Chase)` (`:1457`) → `UpdateCameraPose Chase` (`:2398`) → `SetLookAtDirect`. `MMB/RMB→OnChaseOrbit`, `wheel→OnZoom` (`:1520`).
7. **F4 entrada:** `VK_F4` (`:3512`) → `SetCameraMode(Free)` + `InitFreeCamFromChase` (`:1474`) → `SetLookAtDirect+SetOrbitTarget` → detach (`:2445` no-op).
8. **F4 pick LMB:** `WM_LBUTTONDOWN` → `Remap` → `OnLMouseDown Free` (`:2263`) → `ScreenPointToRay` (`:527`) → `ScreenPointToTerrain` (`PathTool.cpp:985` via `SampleGround :621`) → fallback Z=0 → `SetOrbitTargetSmooth` (`:296`) → `Update blend` (`:42`) → `UpdateCameraPositionFromOrbit` (`:570`).
9. **F4 orbitar:** `MMB/RMB/Alt+LMB→OnMouseMove Free` (`:2909`) → `OnOrbit` (`:62`) → `Clamp+UpdatePosition`; `wheel→OnZoom` (`:114`); `Shift+MMB→OnPan` (`:82`).
10. **Volante teclado:** `A/D GetAsyncKeyState/m_keySteer` (`:1834`) → `input±rate*0.01` (`:1976/1986`, rate §10) → `clamp` → `steering_input` → `VehicleDynamics:81` → `ChassisMotion:773 Ackermann` → `yaw/pose` → `visual/câmera`.
11. **Mouse-drive `O`:** `O borda` (`EditorViewportHost.cpp:3552`) → `m_mouseDrive` → `nx,ny` (`:2854`) → `MouseSteerTarget` (`CockpitDrag.hpp:102`) → `BoostHold/slew` (`:2002,2028`) → `steering_input + pedais analog` → mesma física.
12. **Pedais→longitudinal:** `W/S→held` (`:1846`) → `PedalInputController.Update` (tip-in/kickdown/degraus) → `throttle_raw/brake_raw` → `ControlSystems` → `Drivetrain/BrakePlant` → `VehicleDynamics aero/roll/slope` → `Drivetrain única escrita v` → `odometria`.

---

## 10. Matemática — fórmulas, consts, convenções

- **Eixos VSE:** Z-Up `X=lat,Y=long,Z=height` (`OmsiBusParser.cpp:116-122`, `SimulationPreviewHost.cpp:2326-2327`). openOMSI mundo `glam::DVec3` x east/y north/z up, heading CW de north vs `.cfg` x right/y fwd/z up (`ARCHITECTURE.md:125-139`). `⛔ Risco de conversão — validar eixo a eixo, não assumir identidade`.
- **Graus/rad:** `kPi,kDegToRad` (`EditorCameraController.cpp:9-10`), yaw/pitch graus armazenados, `cos/sin` em `UpdateCameraPositionFromOrbit :571-585` e `UpdateCameraPose`. Sem quaternion. Matriz veículo `R=RotY(roll)*RotX(pitch)*RotZ(-psi)` (`:2492-2501`), `world=toNeutral(-neutralY)*R*T(pos)` (`:2513-2516`). View row-major LH LookAt (`:391-408`), proj persp reverse-Z (`:473-488` `w=h/aspect,h=1/tan(fov/2)`), ortho reverse-Z (`:490-501`).
- **Ordem:** interior `offset+eyeDist*view (25m look)→VehiclePointToWorld`; F3 `offset orbital→base yaw-only→+pivotZ`; órbita genérica `pos=target+dist*[cosP*sinY,-cosP*cosY,sinP]` (`:583-585`), inversa `pitch=asin(dz/dist),yaw=atan2(dx,-dy)` (`:284-291`).
- **Ray:** `ndcX=2x/w-1,ndcY=1-2y/h` (`:530-531`), ortho `origin=pos+right*ndcX*halfW+up*ndcY*halfH` (`:541-546`), persp `dir=fwd+right*ndcX*tan*aspect+up*ndcY*tan` (`:555-563`).
- **F1 consts:** orbit `0.245°/px (0.35*0.70 :1534)`, `zoomStab 25% (:1536)`, clamp `yaw±90,pitch-35..+35 (:1540-1541)`; zoom `364px full, intent 0.70 (:1552-1555)`; FOV `base/(1+5.5z) min 4 (:1440)`; ease `0.54s s=1-(1-t)³ (:1398,1407)`; yaw shortest `remainder 360 (:1377)`.
- **F3 consts:** sens `0.35`, pan `x4`, pitch `-10..75`, dist `4..40`, zoom `1.5*delta`, FOV `60`, `offZ 1.6+dist*sin, tgtZ pivot+1.4`, `tauZ 0.30`, `dist0=wb*1.5+4`.
- **Genérica:** `orbitSens 0.35`, pitch damp `>70° até 0.20 (:70-74)`, zoom `0.88/1.14 clamp 0.1..2000 (:117-118)`, dolly `0.0028 clamp 0.80..1.20, 0.05..2500`, pan `worldPerPixel=2*dist*tan(fov/2)/vpH (:98-99)`, POI `blend 1-exp(-10dt) stop 1e-5 (:42-54)`, `Focus dist=r/sin(fov/2)*1.35 (:177)`, clamp `pitch±88,yaw wrap 0..360 (:590-594)`. Terreno pick `max 4000, dt 0.05..2.0, bisseção 14 (:1003-1036)`.
- **Volante teclado (por substep 0.01s):** `base=0.336*sens, min=0.145*sens, sens 0.70 [0.20-2.00]`, `speed_turn=0.38+0.62e^{-0.045v_kmh}`, `speed_unwind=0.48+0.52e^{-0.032v_kmh}`, `low=2→1 smoothstep 20→45km/h`, `steer_rate=clamp(base*low*turn,min,base*2.2)`, `unwind_base=clamp(base*low*unwind,min,base*2.4)`, pitch Hermite `1.0/1.35/2.15/2.75/3.25`, `v_ramp=clamp(v/1.5,0,1)`, `unsteer` blend, `input±rate*0.01`. openOMSI `KeyboardAxes::update` taxa `0.8/(1+v/45)`, retorno mola, `linear 0.05`, snap `<0.0003` (`input.rs:102`) — **fórmulas diferentes, não equivalentes**.
- **Mouse:** `edge=0.75|nx|, corner=lin(0.20,1,|nx|)*lin(0.10,1,|ny|), mag=edge+(1-edge)*corner, target=±mag`; `thr=(ay-0.10)/0.90, brk=(dn-0.10)/0.60`; `tau 0.12s, slew 2.5/s`. openOMSI `mouse_steering=(2x/w-1)/max(1,kmh/10)*inv_min_turnradius` (`player.rs:2015`) — **fórmula diferente**.
- **Retorno:** `base=0.22*(v/5)e^{-0.020v}`, `half=850 ou cfg*0.5`, `angle=|input|*half`, `f` smoothstep por ângulo (`1.0/>360, 0.20-1.0/45-360, 0.05-0.20/5-45, max(0.015,a/100)/≤5`), `auto=0.33+0.67*pitch`, `step=base*f*auto*dt`. openOMSI mola em `KeyboardAxes` + `VehiclePhysics rate max*2.5*dt` — **diferente**.
- **FFB:** `-0.025*δ*v² clamp ±15 [N·m]`. openOMSI sem equivalente sintético.
- **Ackermann/yaw:** `u=δ/max, inner=|u|max, r_in=L0/tan(inner), r_out=r_in+track, outer=atan(L0/r_out), L0≥1.5,track≥1.2`, `f_curva` tabela, `dirFade 25→40km/h ratio -0.3`, `R=L/(tanδ f)`, `ω=(v/L1)tanδ f clamp ±0.75g/|v|, lerp tau 0.10 clamp ±3.0, ay clamp ±0.65g, v_lat clamp ±4.0`, `Izz=m(L²+track²)/12, Mz=Σ(lyFx-lxFy), κ clamp ±5, α=atan2, elipse ≤1, scrub 0.06`. openOMSI `step()` simples + `rigid` — **sem paridade**.
- **Long:** `F_aero=0.5*1.225*Cd*A*v² (0.40,8-8.5), F_roll=0.006*m*g*cosθ, F_slope=m*g*sinθ, F_brake=pedal*g_max*m*g*air*low*ease (0.385/0.504, low curva, ease lerp 0.95→1 quart v/5), parking=apply*0.65*Fz, v=clamp(-8,70), x+=v dt, heading+=r dt`.

---

## 11. Input — teclado/mouse fluxo completo

Win32→Host (`EditorWindow.cpp:290-305,394-598`, `RemapClientToScene :689`): `WM_LBUTTONDOWN→OnLMouseDown:424, RB→OnRMouseDown:511, MMB→OnMMouseDown:529, MOVE remap:569-572, WHEEL→OnMouseWheel:583, KEYDOWN→OnKeyDown:598`.
Play keys (`EditorViewportHost.cpp:3417-3647` consome tudo): `F1→Driver:3500, F2→Pax:3504, F3→Chase:3508, F4→Free+Init:3512-3518, Space→Notify:3491, Back/Home→Reset:3496, Arrows→return poll:3520 (Update :2162-2220), W/A/S/D→flags:3419-3434, O→mouse-drive borda:3548, RMB desliga mouse-drive:2666`.
Mouse Play (`:2899-2928`): Free ramo `:2901-2912`, interior `MMB Driver→Orbit:2919, RMB→ZoomDrag:2921`, chase `MMB||RMB→ChaseOrbit:2924`. Fora Play `:3111-3128` editor orbit/pan/dolly. Scroll `:3131-3150`: interior no-op, Free `m_camera.OnZoom`, senão `PreviewHost.OnZoom`, fora Play `m_camera.OnZoom`. `m_f1MmbGuardUntilMs=now+180ms (:2683,2819, hpp:450)` `⛔ escrita sem leitura encontrada`.
Contínuo `Update :1830-1837,2162-2220,2837-2857`: `GetAsyncKeyState(WASD/setas/space)`, `GetKeyState(Ctrl/Alt/Shift)`, mouse-drive pos.
openOMSI: `winit→input_script.rs (bindings, F1-F4 :376, O :237, orbit :1164, toggel_mouse_ctrl :2221)` → `EngineAction/KeyboardAxes (input.rs)` → `app_events.rs:586-677,869,952,1277,1365,1378,2062,2277` → `Controls` → `VehicleInstance::update`. Touch separado (`touch.rs→analog`), `OMSI_INPUT/TOUCH/ORBIT_DIST/TRACE_STEER` p/ teste.

---

## 12. Renderer/Ground/Raycast — fluxo câmera↔mundo

VSE: câmera escreve `SetLookAtDirect+SetFov` → renderer lê `GetView/Projection/RelativeView` + `SetCurrentCameraViewpoint(int(mode))` (`EditorViewportHost.cpp:1202`, matrizes `:1204-1211`) + `PickCockpitClickable` filtra `viewpoint==1||4 skip, pax só 0||2` (`:1871-1874,1907-1908`, `Renderer.cpp:4302,5265,5521` único filtro `viewpoint==4`). Ground `m_roadGroundEnv` (`RebuildRoadGroundFromTransform:4654 BuildFromMesh, BindPathTerrain SampleGround, PlaceSpawn SampleGround, RaycastEntity SampleGround :4131,4667,5222`) alimenta `ScreenPointToTerrain`. `PreviewHost::Update(frameDt,m_camera)` (`:1152`) + `GetVehicleWorldMatrix+rideHeight` por frame.
openOMSI: `omsi_render::{Camera,Renderer,Scene}` (`lib.rs:110`), `World::{ground_height (:2316 texel), camera_ground (:2359), walk_height (:2412), walk_height_near (:2453)}`, `cursor_ray (:112)`, `placing::ground_hit (:24 marcha+bisseção 24, usado :80)`, `ray_mesh/triangle/near_sphere (omsi-geometry:1315-1467)`, `player::pick/nearest_hits/surface/body_hit (só bus)`, `camera_arm::free_length (:398 5 raios vs Blocker+ground, MARGIN 0.4/CLEARANCE 0.6) + SpringArm (:527 HOLD 0.35/EASE 3.0/PULL_IN 18/ARM_MIN 1.0)`. `⛔ Sem API unificada; editor pick por proximidade, não ray; sem ray→CollisionWorld/Obb p/ target mundo`.

---

## 13. Vehicle Dynamics — fluxo volante/condução

Ver §2.5 + §8 + §10. Ordem `SimulationPreviewHost.cpp:364-380 ControlSystems→…→BrakePlant→…→Engine→Drivetrain→…→VehicleDynamics→ChassisMotion→…`, `dt=0.01` (`VehicleManager.cpp:48-49`), `clampedDt/accumulator/≤5 substeps (:1819-1843,2140)`. openOMSI `app_events→KeyboardAxes::update(dt)→Controls→VehicleInstance::update(dt):1984 (clock→step_physics→probe→trigger→dirt/engine→vm.run_frame→visuals)`, `dt clamp 0..0.1 (physics.rs:79)`, `SimClock`, AI `par_iter_mut`, `rigid::step/collide` p/ player vs `ai_motion::AiBody` p/ IA. Gameplay gates: portas travam `>0.833m/s`, `throttle=0` se aberta, kneeling off `>2.778m/s`, limpadores `>22.22m/s` (`ControlSystems.cpp:164-184,206-212`), cruise PID `Kp0.5 Ki0.05 Kd0.02 ±2.0 (:75-78,105-124)`, articulado hold/anti-jackknife.

---

## 14. Diferenças arquiteturais

- VSE monolito editor (`SimulationPreviewHost` + `EditorViewportHost` + `EditorCameraController` + Win32 direto, `GetAsyncKeyState` polling) vs openOMSI separado parse (`omsi-vehicle`) → runtime (`omsi-sim`) → jogo (`omsi-app` winit) → desenho (`omsi-render`), sem global mutável (`Arc+Mutex+OnceLock`, `ARCHITECTURE.md:3-6`).
- VSE Z-Up lat/long/height + matriz `RotY*RotX*RotZ(-psi)` + LookAt LH reverse-Z vs openOMSI `DVec3` east/north/up + heading CW north vs `.cfg` right/fwd/up — conversão obrigatória.
- VSE timestep fixo 0.01s accumulator vs openOMSI `dt clamp 0..0.1` + `SimClock` + steps próprios + rayon paralelo.
- VSE F4 respawna de F3 + target `Desired` interpolado vs openOMSI `free` mapcam + `orbit/SpringArm` só externa.
- VSE steering state único (`VehicleState`) + Ackermann/f_curva/direcional/elipse + FFB sintético vs openOMSI distribuído (`KeyboardAxes/Controls/VehiclePhysics/Rigid/Wheel`) física simples.
- VSE `O` duplo sentido por binário vs openOMSI `toggel_mouse_ctrl` único + touch separado.
- openOMSI tem `compatibility first + omsi-check + OMSI_ROOT skip + one change per PR` (`CONTRIBUTING.md`) — VSE sem equivalente.

---

## 15. Gaps 🟢/🟡/⛔

| Item VSE | Status | Evidência openOMSI |
|---|---|---|
| F1-F4 modos | 🟢 existe direto | `input_script.rs:376 F1-F4`, `player::camera_look:1667`, `app_events:869,952` |
| `.bus` positions + `camera_std` | 🟢 existe direto | `vehicle.rs:16,101-107,144,315` |
| Orbit + zoom + colisão externa | 🟢 existe direto | `camera_util ORBIT_*:76 + player clipped:1619 + camera_arm free_length/SpringArm:398,527` + testes |
| `cursor_ray / Camera::ray` | 🟢 existe direto | `camera_util.rs:112` |
| `ground_height/camera_ground/walk_height` | 🟢 existe direto | `scene.rs:2316,2359,2412,2453` |
| `ray_mesh/triangle` baixo nível | 🟢 existe direto | `omsi-geometry:1315-1467` |
| `mouse_drive O` | 🟢 existe direto | `input_script:237 + app:130 + player:2015 + app_events:619` |
| `KeyboardAxes/Controls/VehiclePhysics` steering | 🟢 distribuído | `input.rs:35,102 + physics.rs:16,36,78` |
| Bind/Apply/Wrap/SwitchPax instant | 🟢/🟡 | `camera_look + (std+choice)%n` — checar `Reset home=std` |
| `SwitchDriver ease 0.54s + shortest yaw + cross-unit` | ⛔ não existe | Só `driverview_smooth` setting |
| `OnInteriorOrbit/ZoomDrag consts + InteriorDisplayFov 1+5.5z` | 🟡 parcial | `view_looks/zoom/look` consts diferentes |
| `UpdateCameraPose F3 exata (offZ 1.6/tgt 1.4/tauZ 0.30/yaw-only)` | 🟡 parcial | `camera_look outside + clipped` — reimplementar p/ paridade |
| `InitFreeCamFromChase (respawn F3)` | ⛔ não existe | `free=default_camera mapcam` |
| `ScreenPointToTerrain 14 iterações heightfield` | 🟡 parcial | `ground_hit 24 iterações texel` — unificar |
| Serviço `raycast→GroundHit|TargetHit` único mundo | ⛔ não existe | Cada caller reimplementa; editor pick por proximidade |
| Ray→`CollisionWorld/Obb/MeshShape` p/ target | ⛔ não existe | `collision.rs` sem query ray |
| `MouseSteerTarget + ratchet + slew tau0.12` | 🟡 parcial | `mouse_steering` fórmula diferente |
| Steer integrate teclado (base/exp/low/Hermite) | 🟡 parcial | `KeyboardAxes taxa 0.8/(1+v/45)` diferente |
| Retorno só em movimento + fator ângulo | ⛔ não existe | Mola genérica |
| `Ackermann por roda + f_curva + direcional -0.3 + elipse + scrub` | ⛔ não existe | `step_physics/rigid` simples |
| `FFB -kδv² ±15` | ⛔ não existe | FF nativo + rumble script |
| Pedais tip-in/kickdown/degraus latch | 🟡 parcial | `pedals_as_omsi` teste, sem latch |
| `bump_steer_deg` lido não aplicado | ⛔ evidência insuficiente uso | `ChassisParser.cpp:119` sem uso em `ChassisMotion/VehicleDynamics` |
| Deadzone volante dedicada / damping coluna / low-pass teclado / gamepad eixo→input / curva pow direção | ⛔ evidência insuficiente | Nenhum `deadzone` volante, nenhum `c_volante`, integração direta |

---

## 16. STEP-BY-STEP de implementação (futuro, não executar agora)

Formato `🟢 concluído (só se equivalência provada) / 🟡 pendente-adaptação / ⛔ bloqueado-não-existente`.

- [ ] **Etapa 0 — Higiene.** Objetivo: baseline verde. VSE: —. openOMSI: `cargo test --workspace`, `cargo run -p omsi-check -- "$OMSI_ROOT"` antes/depois (CONTRIBUTING). Risco: ambiente. Verificação: CI verde. Status: 🟡.
- [ ] **Etapa 1 — Enum câmera + `camera_std` home.** Objetivo: `view` tipado + `Reset→std` idêntico VSE (`:1608`). Arquivos VSE `SimulationPreviewHost.hpp:68, cpp:1608`; openOMSI `app.rs:211, player.rs:1679, input_script.rs:376, app_events Space/Home`. Risco: string vs enum quebra script. Teste: `camera_std` + Space volta p/ std, não 0. Status: 🟡.
- [ ] **Etapa 2 — Ease F1 0.54s.** Objetivo: `Begin/AdvanceEase s=1-(1-t)³ + shortest yaw + cross-unit instant (:1363,1402,1566)`. openOMSI: novo estado transição em `omsi-app` (não `sim`), respeitar `driverview_smooth`. Risco: ownership/borrow, `Arc` câmera. Teste: troca ←/→ mede 0.54s + yaw curto. Status: ⛔.
- [ ] **Etapa 3 — Look/zoom interior consts.** Objetivo: `0.245°/px, clamp yaw±90 pitch-35..+35, zoom 364px, FOV base/(1+5.5z) (:1531,1544,1435)`. openOMSI `view_looks/zoom`. Risco: FOV diverge. Teste: pixel→grau + FOV 6.5x. Status: 🟡.
- [ ] **Etapa 4 — F3 exata.** Objetivo: `offZ 1.6/tgt 1.4/tauZ 0.30/yaw-only/dist0=wb*1.5+4/clamp 4..40/pitch -10..75/sens 0.35 (:2398,1520,hpp:277)`. openOMSI `player outside + SpringArm`. Risco: coordenadas. Teste: pose frame a frame vs VSE. Status: 🟡.
- [ ] **Etapa 5 — Serviço picking unificado (desktop-only `omsi-app`).** Objetivo: `cursor_ray + ground_height/camera_ground + ray_mesh` → `picking.rs::raycast→GroundHit|TargetHit`, portar `ScreenPointToRay (:527)` + `ScreenPointToTerrain 14 it (:985)` + `SampleGround (:621)` + fallback Z=0 + Alt+MMB mesh/esfera (`:2698-2809`). Não reutilizar `player::pick` (só bus) nem `camera_arm` genérico. Risco: perf (BVH, `TriBvh::ray`), threading rayon. Teste: clique chão/mesh → ponto 3D ±ε. Status: ⛔ (blocos `CollisionWorld` ray).
- [ ] **Etapa 6 — F4 respawn+orbit.** Objetivo: `InitFreeCamFromChase (:1474) + detach (:2445) + OnOrbit/Pan/Zoom/Dolly + SetOrbitTargetSmooth blend 1-exp(-10dt) (:296,39)`. openOMSI `free/default_camera + orbit`. Risco: confundir com mapcam. Teste: F4 inicia de F3, LMB redefine alvo, não segue bus. Status: ⛔ (depende Etapa 5).
- [ ] **Etapa 7 — Steering integrate + retorno + mouse-drive fórmulas.** Objetivo: portar `base/exp/low/Hermite/slew tau0.12/ratchet/retorno só em movimento/fator ângulo (§10)` para `KeyboardAxes/Controls` sem quebrar touch (`touch.rs→analog` separado). Risco: timestep 0.01 vs `dt clamp`, sens `0.70`. Teste: `input.rs` testes `steering_*` + `mouse_steers_less` + trace `OMSI_TRACE_STEER`. Status: 🟡/⛔.
- [ ] **Etapa 8 — Ackermann/f_curva/direcional/elipse.** Objetivo: portar `ChassisMotion:773-1467 + CurveAuthority + PlanarKinematics + WheelTirePlant` para `step_physics/rigid` atrás de feature flag, sem quebrar `ai_motion`. Risco: maior — física, `Izz/Mz`, articulado hold, `SteeringIcrPivot` proibido p/ yaw. Teste: `collision/rigid/vehicle` + `omsi-check` mapa stock. Status: ⛔.
- [ ] **Etapa 9 — Pedais + FFB.** Objetivo: tip-in/kickdown/degraus + `FFB -kδv²` como camada `omsi-app` (não `sim`), `dinput/evdev_ff` intactos. Risco: FFB nativo vs sintético. Teste: pedais rampa + torque clamp ±15. Status: 🟡/⛔.
- [ ] **Etapa 10 — Visual volante/rodas.** Objetivo: `axle_steering/cp_lenkrad/wheel_rotation` → `driver::Wheel + anim` com `clamp/delay alpha/slew` (`OmsiTransformRig:199`). Risco: nomes OMSI. Teste: `driver` + offscreen. Status: 🟡.

Nenhuma etapa marcada 🟢 — equivalência ainda não provada por teste.

---

## 17. Ordem correta de implementação (respeitar dependências)

`0 higiene → 1 enum/std → 3 look/zoom → 4 F3 → 5 picking → 6 F4 → 2 ease F1 (pode após 1) → 7 steering → 9 pedais/FFB → 10 visual → 8 Ackermann por último (maior risco, depende de 7/9)`.
Não implementar camada antes de `ground_height/ray_mesh/cursor_ray` (Etapa 5 antes de 6) nem `Controls` antes de `KeyboardAxes` (7 antes de 8). Tudo desktop-only em `omsi-app` (`cfg(not(android))` se necessário), nunca em `sim/vehicle/render/geometry`.

---

## 18. Plano de testes — paridade com VSE

- `cargo test --workspace` + `cargo build --release` sempre (CONTRIBUTING). `omsi-check` antes/depois em mapa stock.
- Câmera: estender `camera_arm.rs:583-722` + `lib.rs:560` + novos `player/picking` testes: `WrapCameraIndex` loop, `home=std` Space, ease 0.54s `s=1-(1-t)³` + shortest yaw, look `0.245°/px` clamps, zoom `364px` FOV `/(1+5.5z)`, F3 `dist0/tauZ/offZ`, F4 LMB→ponto (±ε vs `SampleGround`), Alt+MMB mesh/esfera/Z=0, orbit/yaw wrap, `ORBIT_MIN/MAX` vs `4..40`.
- Steering: `input.rs:195,218,258,299` + `player.rs:2053` + `OMSI_TRACE_STEER/INPUT/ORBIT_DIST/DEBUG_CAMERA`: taxa vs velocidade, retorno só em movimento, mouse `edge/corner/slew`, pedais rampas, FFB clamp, Ackermann `r_in/r_out` + `f_curva` tabela + direcional `>40` trava + elipse, `bump_steer` ainda sem uso.
- Offscreen `offscreen.rs:966,1382,1648` + `OMSI_ROAD_PHOTO` p/ regressão visual. Testes com `OMSI_ROOT` dão skip sem instalação.
- Critério: Etapa só 🟢 após teste provar equivalência numérica, não conceito semelhante.

---

## 19. Riscos

- **Coordenadas:** VSE Z-Up lat/long vs openOMSI east/north/up + `.cfg` right/fwd/up + heading CW north — erro silencioso de sinal/eixo. Mitigar com matriz teste eixo a eixo.
- **Timestep:** VSE fixo 0.01s accumulator ≤5 vs openOMSI `dt clamp 0..0.1` + `SimClock` + rayon — taxas `rate*dt` divergem se `dt` não fixado em teste.
- **Input:** Win32 `GetAsyncKeyState` polling + borda `m_prevKey*` vs winit eventos + `OMSI_INPUT` script — repeat/borda `O/Space` sensível.
- **Renderer:** LookAt LH reverse-Z vs `wgpu 29` — FOV/near/far (`0.05/2000` vs openOMSI) altera pick `ray`.
- **Raycast/collision:** heightfield 14 it vs texel 24 it vs BVH `TriBvh::ray` vs `CollisionWorld/Obb` sem ray — alvo F4 pode flutuar ±cm-m sem unificação.
- **Camera state:** `m_isCamTransitioning/start/target/unitIndex/ZFilt/BoostHold` sem equivalente — ownership `Arc/Mutex` + `SpringArm` HOLD/EASE podem mascarar ease VSE.
- **Vehicle dynamics:** Ackermann/f_curva/direcional/elipse/scrub/Izz vs `rigid` simples — portar parcial causa spin/ understeer divergente; `IcrPivot` proibido p/ yaw.
- **Threading/ownership:** tile/streamer/rayon/`Mutex<Arc<CollisionWorld>>` vs VSE single-thread editor — contenção `parking_lot`, `apply_texture_upgrades`, borrow checker em transição câmera.
- **Desktop/mobile:** `cfg(android)`, `OMSI_TOUCH/MOBILE`, `arboard/rfd` not-android — implementar em `sim/render` vaza p/ mobile; manter em `omsi-app`.
- **Toolchain:** stable ≥1.85 não pinned — `glam 0.30/wgpu 29` sensíveis a versão.

---

## 20. Critérios de paridade (objetivos, verificáveis)

- **F1:** N posições `.bus` carregadas (incl. trail `unitIndex=1`), ←/→ wrap, cross-unit instant, mesmo-unit ease 0.54s `s=1-(1-t)³` + shortest yaw, look `0.245°/px` clamps, zoom `364px` FOV `/(1+5.5z)` min 4, `Space→std` (não 0), cabeça difere F2 (F2 instant sem orbit).
- **F2:** mesmas posições `pax`, navegação wrap instant, zoom RMB igual, sem `OnInteriorOrbit`, pick filtra `0||2`.
- **F3:** `dist0=wb*1.5+4`, yaw 0 pitch 12, `offZ/tgtZ/pivot tau 0.30`, orbit sens 0.35 pitch -10..75, zoom `1.5*delta` 4..40, FOV 60, segue veículo yaw-only, setas x4.
- **F4:** inicia de F3 (nunca resume), detach (não segue), LMB heightfield+bisseção 14 ou Z=0 → `Smooth blend 1-exp(-10dt)` vira alvo, Alt+MMB mesh→esfera→Z=0, orbit/pan/dolly/zoom dist 0.1..2000 pitch ±88, setas pan 10px/frame.
- **Volante:** `input [-1,1]` integra `rate*0.01` com `base/exp/low/Hermite` + `sens 0.70 [0.20-2.00]`, mouse `edge/corner/slew tau0.12/ratchet`, retorno só em movimento com fator ângulo + auto, `wheel=input*half (850 ou cfg/2)`, `δ_road=input*max (50°)`, clamp `[-1,1]`.
- **Condução `O`:** borda sem repeat, semeia `Sm=input` sem teleporte, `nx,ny→steer/thr/brk` fórmulas §10, RMB desliga, WASD ignorado enquanto ativo, standalone `O` luz (não direção) fora de escopo Play.
- **Física:** `Ackermann inner/outer + f_curva tabela + direcional -0.3 fade 25→40 + R_bicycle + κ/α/Fx/Fy/elipse/scrub + Mz/Izz + r lerp tau0.10 clamps + ay/v_lat clamps + heading/pos integra + hold/anti-jackknife`, longitudinal `F_aero/roll/slope/brake/parking + v clamp [-8,70] única escrita`, pedais rampas, FFB `-kδv² ±15` (último `ControlSystems` vence), timestep 0.01.
- **Geral:** `cargo test --workspace` + `omsi-check` verdes, sem quebra mapa stock/mod, desktop-only em `omsi-app`, sem bump toolchain/framework, sem `⛔ EVIDÊNCIA INSUFICIENTE` restante sem marcação explícita.

---

## Apêndice — fontes primárias verificadas (existência confirmada em disco)

VSE: `src/editor/SimulationPreviewHost.hpp (27375 B)`, `src/editor/EditorCameraController.hpp (6438 B)`, `src/parsers/OmsiBusParser.hpp (7156 B)`, `src/dynamics/ChassisMotion.cpp (124566 B)`.
openOMSI: `crates/omsi-vehicle/src/vehicle.rs (26159 B)`, `crates/omsi-app/src/player.rs (99164 B)`, `crates/omsi-sim/src/input.rs (15962 B)`, `crates/omsi-app/src/camera_arm.rs (25001 B)`.
Demais `arquivo:linha` citados foram localizados via busca código; se divergência for encontrada na implementação, prevalece o código e este doc deve ser corrigido, nunca o contrário.

> Fim da auditoria. Próxima etapa (fora deste doc): ler este MD, confirmar cadeias `VSE→Renderer/Ground/Raycast→Camera` e `VSE→Input→VehicleDynamics→Steering`, e só então planejar `steering-camera-v2` mecanicamente a partir daqui.

---

## Implementation Result (implementação executada nesta sessão, sem commits)

Leitura integral deste MD + VSE citados antes de qualquer edição. Toolchain inalterada
(`rust-version 1.85`, sem bump, sem framework novo). Todas as alterações no working tree,
sem commit, para revisão.

### Arquivos criados

- `crates/omsi-app/src/vse.rs` (novo, desktop-only em uso): matemática pura VSE com
  testes — ease `0.54s s=1-(1-t)^3`, `WrapCameraIndex`, shortest-yaw, `InteriorDisplayFov`
  `base/(1+5.5z)`, orbit/zoom interior, `chase_dist0=wb*1.5+4`, `chase_orbit sens 0.35`,
  `chase_zoom 1.5x 4..40`, `chase_offset offZ 1.6`, `chase_z tau 0.30`,
  `f4_blend 1-exp(-10dt)`, mouse `edge/corner/slew tau 0.12/2.5s` + ratchet + pedais
  `0.10`, teclado `base 0.336*sens/speed exp/low/Hermite`, retorno só em movimento,
  FFB `-0.025δv² ±15`, Ackermann `r_in=L/tan(inner)`, `R=L/(tanδ·f)`, `f_curva`,
  direcional `-0.3 fade 25→40`, yaw-target. 10 testes.
- Registrado em `crates/omsi-app/src/lib.rs` (`mod vse;`).

### Arquivos modificados

- `crates/omsi-app/src/app.rs`: `CAM_BLEND_SECS 0.6→0.54` (= `VSE_CAM_EASE_SECS`);
  `CamBlend::progress` smootherstep → ease-out VSE `1-(1-t)^3`; novo estado
  `vse_free_goal: Option<(yaw,pitch)>` (retarget F4, desktop-only).
- `crates/omsi-app/src/camera_util.rs`: `ORBIT_MIN 3.5→4.0` (VSE `4..40`); `ORBIT_MAX`
  já era `40.0`.
- `crates/omsi-app/src/player.rs`: vista externa F3 com paridade VSE
  (`yaw = heading+look`, `pitch = (12+look) clamp -10..75`, `FOV 60`, `dist 4..40`);
  `camera_look` mantém assinatura (sem quebrar callers); helpers
  `vse_mouse_steering`/`vse_mouse_pedals` (OMSI `mouse_steering` preservado p/ teste).
- `crates/omsi-app/src/input_script.rs`: `look_by` interior `yaw ±90/pitch -35..+35`
  (VSE `OnInteriorOrbit`), exterior pitch `-10..75` com wrap 360; `zoom_by` mínimo
  `0.2→0.154` (`1/6.5`, VSE full-zoom); F4 (`view_set_map`) respawna da pose F3
  (`camera_look("outside")`, = `InitFreeCamFromChase`+detach) em vez de mapa fixo
  `25m/30m/-45°`; `on_left` F4 retarget: Alt → `surface_hit`/`body_hit` (mesh/Moeller
  via `ray_mesh`), senão `vse_ground_hit` + fallback `Z=0`; sem Shift.
- `crates/omsi-app/src/app_events.rs`: drive-mode `O` com matemática VSE
  (`nx,ny` → `vse_mouse_target` + `vse_boost_hold` em `mouse_edge` + pedais VSE com
  deadzone `0.10` + lag `tau 0.12`/slew cap `2.5/s` + fade 1s sem teleporte);
  `mouse_kmh` segue atualizado p/ diagnóstico; trace `OMSI_TRACE_STEER` agora grava
  `nx,ny`; free-camera aplica `vse_free_goal` com `vse_f4_blend`.
- `crates/omsi-app/src/placing.rs`: `ground_hit` com marcha VSE
  (`t=0.05`, passo `clamp(0.75/horiz,0.05,2.0)`, 14 bisseções, `max 4000`) + novo
  `vse_plane_fallback` (`Z=0`); assinatura preservada.
- `crates/omsi-app/src/lib.rs`: init `vse_free_goal: None`.
- `crates/omsi-sim/src/input.rs`: novo campo `steering_sens` (default `0.70`);
  `vse_keyboard_step` (integração `rate*dt` VSE) + `vse_return_to_centre` (só em
  movimento) + matemática espelhada (sim segue independente de app); `update`
  OMSI original intacto; 1 teste novo.
- `crates/omsi-sim/src/physics.rs`: novos estados `wheel_deg` (`input*850`),
  `ffb_nm` (fonte única `-0.025δv² ±15`), `ackermann_rad`, `bicycle_radius_m`
  preenchidos em `step`; helpers `ffb_nm/ackermann/bicycle_radius/curve_authority/
  directional`; dinâmica longitudinal/ Marin intacta.

### Dependências adicionadas

Nenhuma. Só infra existente (`glam`, `winit`, `omsi-geometry::ray_mesh`,
`World::ground_height`, `Player::surface_hit/body_hit`).

### Gaps resolvidos / adaptações inevitáveis

- 🟢→implementado: ease F1 `0.54` (era ⛔), look/zoom consts (era 🟡),
  `ORBIT_MIN 4.0` + F3 `0/12/-10..75/FOV60` (era 🟡), F4 respawn+retarget LMB/Alt+MMB
  + smoothing (era ⛔), `ground_hit` 14 iterações + `Z=0` (era 🟡),
  mouse `edge/corner/slew/ratchet/pedais` (era 🟡), teclado VSE + retorno em
  movimento (era 🟡/⛔), FFB fonte única + Ackermann/`f_curva`/direcional/bicycle
  (era ⛔).
- Adaptações registradas (sem mudar semântica): `look_by` recebe graus (caller escala
  `0.15°/px`), então sens VSE vive nos clamps/curvas, não no ganho do drag;
  `view_zoom` segue multiplicador (`min 0.154 ≈ 1/6.5`) em vez de `zoomAmount 0..1`;
  `mouse_steer`/`mouse_edge` reutilizados como `SteerSm`/`BoostHold`; eixos openOMSI
  (`x east/y north/z up`, heading CW north) preservados — offsets VSE aplicados no
  frame yaw-only do veículo, sem swap de handedness; `Space` já voltava a
  `cam_choice=(0,0)` ≡ `camera_std` (confirmado, sem mudança); F2 já era instantâneo
  (`CamBlend` só driver, sem mudança); `touch.rs`/mobile/VR intocados.
- Permanece ⛔ (declarado, não mascarado): `bump_steer_deg` lido sem uso no VSE
  (sem equivalente a portar); damping dedicado de coluna e deadzone de volante
  (VSE não tem — só limiares); elipse de pneu/scrub/`Mz/Izz`/yaw-lerp `tau 0.10`
  completos do `ChassisMotion` (portado o núcleo Ackermann/bicycle/direcional/FFB;
  acoplamento planar total segue no modelo simples `step/rigid` + `ai_motion`
  para não quebrar mapas/gameplay); pedais tip-in/kickdown/degraus latch (VSE
  `PedalInputController`) não portados — pedais seguem `KeyboardAxes` + analógico
  do mouse VSE.

### Testes executados e resultados

- `cargo check --workspace`: ok.
- `cargo check --tests --workspace`: ok.
- `cargo test -p omsi-sim`: ok — 174 passed, 0 failed.
- `cargo test -p omsi-app --lib`: ok com `CARGO_INCREMENTAL=0` — 147 passed,
  0 failed (inclui 10 novos `vse::tests` + `mouse_tests` OMSI preservados).
  Nota: sem a env, o link do binário de teste `omsi-app` falha nesta máquina
  Windows com `LNK2019` em rlibs de terceiros (`omsi-net`/`launcher_lib`,
  símbolos LLVM `tungstenite/zip/serde`) — infra incremental/MSVC, pré-existente e
  alheia ao diff (só Rust puro + deps existentes); `cargo check` passa sempre.
- `cargo test --workspace` (com `CARGO_INCREMENTAL=0`): exit 0, nenhum
  `test result: FAILED`.
- `cargo build --release`: ok (`Finished release profile`, ~5m35s).
- `cargo fmt --check`: único diff é o pré-existente `crates/omsi-app/build.rs`
  (não tocado); arquivos do diff estão fmt-limpos.
- `OMSI_ROOT`/compat (`omsi-check`, mapa stock) não executado — sem `OMSI_ROOT`
  nesta máquina (testes com `OMSI_ROOT` dão skip por construção).


---

## Auditoria de input (continuação)

A paridade da árvore de input (WASD drive, setas=câmera, LShift/LCtrl câmbio automático, MMB orbit, RMB zoom F1, wheel só F3, Alt+MMB pick F4) foi auditada e implementada em documento próprio: docs/VSE_INPUT_PARITY_AUDIT.md (metodologia idêntica a esta auditoria, VSE como fonte da verdade, sem tocar Android). Este documento permanece a referência de câmera/steering/dynamics; o de input é a referência de bindings/contexto/prioridade.
