# F3/F4 — Autópsia da trava central (auditoria, sem correção ainda)

> Regra da fase: NENHUMA correção de offset/pivot/clamp até a causa nascer com
> nome e sobrenome. Steering portado intacto; renderer/Android/VR intactos.
> HEAD congelado: `20131cd`; working tree preservado (sem reset/stash).

## Fase 1 — Estado congelado

- Commit: `20131cd Changelog: wheel seating, #406, #407, #408`.
- 13 arquivos modificados + `vse.rs` + 2 docs + 1 junk (`witch -c ...`, reflog
  capturado por acidente — não tocar).
- `openOMSI-recovered` intocado. Steering das fases anteriores intocado.

## Fase 2 — Cadeia real F3 (input → pixel)

```text
F3 key (DIK 61/stock `view_set_outside`, ou `F3` hardcoded `input_script.rs:376`)
→ on_key → game_action("view_set_outside") → view="outside"
→ frame (app_events.rs): sync_view_look → camera_look("outside", base, look, orbit)
  [player.rs: outside branch — pivot DE `Self::stable_pivot` (yaw-only),
   yaw=heading+look.0, pitch=-(12+look.1), FOV 60, pos=pivot−forward·dist(4..40)]
→ camera_clipped ( MESMO pivot; want=orbit; back=−forward;
  free=free_length(world, pivot, back vs cenário + loop de chão GROUND_CLEARANCE 0.6;
  len=SpringArm.update(want, free, pivot, dt); pos=pivot+back·len)
→ renderer.render(scene, camera) — `Camera` é só pose (pos/yaw/pitch/fov);
  view/proj derivadas no `omsi-render` intocado.
```

Escritores de `look` em `outside`: `look_by` (MMB/RMB-drag, Alt+IJKL, pad),
setas do frame (orbit yaw wrap + `P=12+look.1`), `sync_view_look` (troca de
view), reset. Nenhum escreve `position` direto.

## F4 — cadeia real (provado: NÃO é "F3 + free")

```text
F4 key (DIK 62 / `view_set_map`) → game_action: snapshot camera_look("outside")
  → view="free", ego=false  (pose herdada; DAQUI em diante nada referencia o bus)
→ frame free: WASD+QE/arrows voam (`app_events.rs` fly), look_by gira yaw/pitch,
  wheel = dolly, LMB = vse_ground_hit→Z=0→vse_free_goal, Alt+MMB = mesh→ground→Z=0,
  RMB-drag = ZoomPrecision, vse_free_goal eased com 1−exp(−10dt).
```

Detach estrutural: o ramo `free` nunca lê `player.vehicle`. Segundo aperto de
F4 hoje = re-snapshot (não há FlyMode separado — a construir na Fase 10).

## Fase 3 — A trava, por eliminação de código

Candidatos e vereditos estáticos:

1. `camera_blockers` (`scene.rs:7370`) — só cenário em tiles, NUNCA o próprio
   ônibus. Não é o ônibus se bloqueando.
2. `free_length` ground loop (`camera_arm.rs:460-504`) — só encurta o braço se o
   raio pivot→câmera MERGULHAR a `0.6 m` do solo. Com pitch −12° o raio SOBE:
   sem hit. Com pitch positivo (bug antigo, já corrigido) o raio DESCE: pino
   em `ARM_MIN` junto ao pivot — exatamente o sintoma relatado.
3. `SpringArm::update` — só obedece ao `free` (teste `arm_pins_only_when_free_
   reports_blocked`); `jumped` (>25 m) dá snap, sem grude.
4. `stable_pivot` — yaw-only, sem bounce; `position.z` = média do terreno
   (`vehicle.rs:1406-1408`), sem bounce de alta frequência.
5. `look` stale — `view_looks` é por sessão (sem persistência); primeira entrada
   em F3 parte de `(0,0)` → `P=12`, pitch −12. Sem contaminação entre sessões.

Conclusão estática: **nenhum pino estrutural restante no caminho nominal** —
o pino histórico era o pitch com sinal trocado (Fase anterior). Resta provar
ao vivo (Fase 4/8) em vez de remendar constantes.

## Fase 4/5 — Diagnóstico vivo (sem GUI: offscreen + env)

- `OMSI_DEBUG_CAMERA=1` loga por frame o que parou o braço (`camera_arm.rs:45+`).
- `OMSI_TRACE_STEER=<csv>` + `<csv>.tick` para o volante (inalterado).
- Offscreen exercita o caminho vivo exato (`offscreen.rs:966-969`:
  `camera_look("outside")` + `camera_clipped(dt=0)`): renderizar F3 com bus real
  e LER o PNG + o log do arm. Teste A (pré-arm) = F4 snapshot / `dt=0` sem
  clip; teste de writers = grep acima (um escritor por estágio).

## Fase 6 — Retrocesso (leitura)

- `camera_arm.rs` (SpringArm/free_length): arquitetura ORIGINAL (`7aaf933`
  primeira estrutura pública) — o "puxar para o pivot quando bloqueado" é
  desenho original anti-parede, não regressão.
- `player.rs` chase (`heading−35`, `pitch −15`, `body_rotation`, `ORBIT_MIN 3.5`):
  base HEAD `20131cd` — câmera ACIMA (pitch negativo), estética própria.
- Fases nossas: pitch `+12` (inversão — o pino), depois `−(12+look.1)` +
  `stable_pivot` + `dist=wb·1.5+4` (correção estrutural, atual).
- Não há "commit da trava" posterior ao HEAD: a trava percebida = pitch
  invertido da fase intermediária + arm agindo sobre geometria errada.

## Fase 7 — Base saudável

A base atual (pós-correção de sinal) É a base saudável candidata: pivot
yaw-only, alno `.bus`, pitch para baixo, distância por wheelbase, arm só
anti-colisão. Validação = offscreen + `OMSI_DEBUG_CAMERA` (Fase 8), não mais
código às cegas.

## Fase 8 — Tabela causal VSE × base (paridade de comportamento)

| Comportamento | VSE (`SimulationPreviewHost.cpp`) | Base atual | Divergência |
|---|---|---|---|
| pivot | `veh − ride`, yaw-only, `pivotZ` tau 0.30 | `stable_pivot` yaw-only, `z` = terreno médio + `outside_center.z` | sem tau (terreno já suave; `SpringArm` dá easing) |
| target | `(vehX, vehY, pivotZ+1.4)` | `.bus outside_center` (SD80 real: `0,0,1.2`) | bus-autorado ≈ +1.4, aceito |
| yaw | chase yaw 0 + heading | `heading + look.0` | igual |
| pitch | +12 (câmera acima, olhando p/ baixo) | `−(12+look.1)` (sinal convertido p/ convenção openOMSI) | conversão documentada |
| dist | `wb·1.5+4`, `4..40` | idem (`vse_orbit_wb`, `ORBIT_MIN/MAX`) | igual |
| orbit MMB | `OnChaseOrbit` 0.35 | `look_by` + mesma estrutura | ganho do chamador difere (item ao vivo) |
| zoom wheel | `dist −= 1.5·delta` | `orbit −= 1.5·amount` | igual |
| zoom RMB | sem equivalente (só F1/F2) | extensão `ZoomPrecision` base 60 | extensão declarada |
| ground | `SampleGround` no pick; sem clamp de pose | `free_length` + `SpringArm` anti-solo/parede | adaptação anti-clip |
| F4 entrada | `InitFreeCamFromChase` + detach | snapshot `outside` + `view=free` | igual em estrutura |
| F4 orbit/pick | LMB terrain+Z=0; Alt+MMB mesh→esfera→Z=0 | idem (`vse_ground_hit`, `surface_hit`) | igual |
| F4 fly | n/a (editor à parte) | WASD+arrows no `free` | modo duplo a explicitar (Fase 10) |

---

## Fase 9/10/11 — Reconstrução (evidência offscreen + estados explícitos)

Offscreen headless com SD80 real em Grundorf (`--view outside`,
`OMSI_DEBUG_CAMERA=1`): traseira → `--look 90,0` lateral → `--look 180,0`
dianteira — órbita 360° íntegra, altura/distância corretas, sem pino, sem
chão, sem log de bloqueio do braço. **Demolição recusada pela evidência:**
a cadeia atual já é a geometria VSE; o "trava" era o pitch invertido da fase
intermediária (+ arm agindo sobre geometria errada).

Implementado (só o autorizado):

- F4 em dois estados explícitos (`App.free_fly`, máquina `vse_free_press` +
  teste): 1º aperto = snapshot VSE + modo órbita (SEM translação WASD —
  como o VSE, que só orbita/zooma/picka); 2º aperto = fly openOMSI
  (WASD/QE movem, mesma pose); 3º = volta ao modo órbita. `on_foot` F4 entra
  voando (walker espera). Ego/pedestre e mapa sem bus voam como antes.
- F3/F4 Space, targeting, RMB/wheel, `camera_clipped`: intocados.
- Steering, F1, F2, física, renderer, backend, Android/VR: intocados.

---

## Fase 9b/10b/11b — RECONSTRUÇÃO TOTAL (demolição autorizada e executada)

Evidência offscreen posterior + autorização expressa: F3/F4 refeitas do zero
sobre o VSE canônico, sem reutilizar a lógica legada.

### Demolido (com prova de exclusividade por grep)

- `player.rs`: `camera_clipped`, `arm`, `stable_pivot` + testes.
- `camera_arm.rs` (arquivo inteiro: `SpringArm`, `free_length`, `Blocker`,
  `TriBvh`, `ray_object`, `OMSI_DEBUG_CAMERA`) + `mod`.
- `scene.rs`: campo `camera`, `camera_shape`, campo `blockers`, push,
  `camera_blockers`, `camera_ground` (só o braço usava; `walk_height*`
  do pedestre intactos).
- `placing.rs`: `vse_ground_hit`/`vse_plane_fallback` (marcha agora vive em
  `vse_orbit`; `ground_hit` do spawner intacto).
- `vse_free_goal` + bloco de easing + `vse_orbit_wb` (máquinas assumem).
- `ORBIT_MIN/MAX` (máquinas clampam sozinhas).

### Construído (`vse_orbit.rs`, zero código legado)

- `VseChase`: `Start`/`ensure_bus`/`enter`/`reset`/`on_orbit_px`/`on_zoom`/
  `nudge_deg`/`pose` (filtro `tau 0.30`, base yaw-only, off/target VSE).
- `VseFreeOrbit`: `update` (glide `1−exp(−10dt)` + reposição),
  `on_orbit(dy,dx)` (damping >70°, clamp ±88, wrap), `on_pan` (1:1 exato),
  `on_zoom` (0.88/1.14), `on_dolly`, `set_target` (+resync, nunca volta),
  `set_target_smooth`, `set_look_at`, `seed_from_chase`, `view_angles`
  (derivados dos pontos), `basis` (com fallback polar), `nudge`.
- `pivot_base`, `chase_points`, `vse_pick_ground` (marcha 0.05–4000, 14
  bisseções), `vse_plane_fallback`, `chase_ease_secs`, `vse_free_press`.
- 12 testes determinísticos com números deduzidos à mão do VSE.

### Fiação (só F3/F4; F1/F2/steering/física/renderer/Android/VR intactos)

- Frame F3: pose da máquina (`ensure_bus` + `pose`), ângulos derivados dos
  pontos; sem `clipped`, sem arm, sem pull-in (VSE não tem).
- Frame F4-VSE: `update` + pose da máquina; fly/ego/sem-bus inalterados.
- MMB: F3 `on_orbit_px` raw; F4-VSE gestos com modificadores
  (Ctrl=dolly, Shift=pan, Alt+RMB=dolly, demais=orbit `(dy,dx)` VSE).
- Wheel: F3 `on_zoom`, F4-VSE `on_zoom`; resto pela tabela (F2 legado intacto).
- Setas/look/pad/Alt+IJKL em F3: `nudge_deg` na máquina; `=/-` e telefoto
  removidos (VSE não tem); Space/Home/direction-reset: `chase.reset()`
  (distância preservada, como o VSE).
- F4: seed da máquina do chase; LMB/Alt+MMB → `set_target_smooth` (mesh via
  `surface_hit`, marcha, `Z=0`); 2º aperto alterna fly com re-ancoragem sem
  salto; `on_foot` F4 entra voando.
- Offscreen F3: núcleo sem estado (equivale ao 1º frame); sem clip.
- `OMSI_INPUT orbit` → máquina; campo `orbit` legado mantido (touch escreve
  nele — mobile intocado por regra).
