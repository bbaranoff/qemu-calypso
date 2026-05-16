## Ce qu'on a (capabilities vérifiées par les tests)

**Observabilité & infra — solide partout (~50 tests verts)**
- Monitor HMP de QEMU réactif (info chardev/mtree/qtree/irq/registers)
- GDB stub fonctionnel (handshake, lecture PC/registres ARM, mémoire DSP DARAM)
- Health pipeline (process attendus présents, pas de zombies, pcap mobile grossit, log frais)
- Filesystem (espace dispo, fd usage, log <2GB)
- Network (ports QEMU en listen)
- VTY mobile (joignable, IMSI loaded, état MM = null/idle)
- 27/30 grep-blockers négatifs (pas d'OOM, pas de panic, pas de SP catastrophe, pas de runaway DSP, pas de bts shutdown clock-skew, etc.) — ton filet de sécurité

**Injection NDB — clean 19/19**
- Écriture FB, SB, SI (1-6), A_CD (23B/30B), D_burst, D_task (24/28/5/6) → tout passe
- `probe_ndb` et `clear_ndb` OK
- C'est ton chemin "écriture côté bridge" qui est validé end-to-end

**DSP côté observation — passe**
- `intm_reaches_zero`, `no_enter_7740_dwell`, `no_wait_a21a_on_window`, `d_fb_det_data_no_longer_zero`, `dsp_throughput_above_threshold`, `bsp_daram_write_distribution`
- POPM decoder actif, tier-A decoder fixes présents, `dsp_throughput_5x`
- → côté décodage observable, le DSP fait son taf

**IrDA debug channel — 7/8**
- PTY existe, lisible, capture process alive, produit des bytes, timestamp prefix, throughput sous saturation, n'interfère pas avec UART_MODEM
- Seul manque : `boot_marker_present` (xfail — le `=== fw-irda boot OK ===` rate la race window de capture)

**Bridge L1-data**
- DL lookahead respecté, trafic visible dans bridge.log
- Bursts DL reçus de bts-trx, FB pattern dominant, pas de clock skew shutdown

**Osmocon dialog**
- L1CTL_RESET_REQ, L1CTL_FBSB_REQ tentés, lost ratio acceptable, task24 dispatched, BSP DMA actif

**Layer 1 control partiel (3/6 runtime)**
- `a_cd_writes_nonzero`, `a_cd_write_pc_includes_ccch_demod`, `neigh_pm_req_loop_alive` → l'ARM lit la NDB et passe par le démodulateur CCCH au bon endroit

## Ce qu'on n'a pas

**DATA_IND propagé vers le mobile (mur historique, 3 NOK)**
- `test_l1ctl_data_ind_received` (milestone + runtime)
- `test_l1ctl_data_ind_rate_vs_alc`
- L'ARM lit, mais ne forward jamais le résultat en L1CTL DATA_IND. C'est le blocker MVP — tout le L3 en cascade.

**Timers TDMA virtuels (2 NOK + 3 NOK drift)**
- `tdma_period_virtual_close_to_target` — virtual clock ne bat pas à 217 Hz
- `kick_realtime_cadence` — wall kick sous le target 200/s
- `bridge_qfn_tracks_qemu_fn`, `bridge_qfn_advances_steadily`, `log_start_within_10s`, `qemu_insn_rate_cv_below_0_4` — QFN bridge désynchro de qemu.fn
- → c'est ton vrai chantier du moment (le "après l'assembleur, le temps réel")

**DSP boot complet (1 NOK)**
- `test_grep_qemu_dsp_booted` — marker "DSP booted" absent du log → boot DSP partiel, IDLE à 0xa21a

**Émulation DSP correcte côté IRQ (6 xfail, voie-2)**
- `c54x_interrupt_ex_called_nonzero`, `isr_entered_implies_rete`, `no_pending_irq_gated` (milestones)
- `interrupt_ex_called_counter_exposed`, `isr_entered_matches_rete`, `no_pending_irq_gating` (runtime)
- → R&D long terme, pas bloquant MVP

**FB detection true emulation (2 xfail/skip)**
- `fb0_att_nonzero`, `synth_zero_path_active` — voie-2 (la vraie démod DSP, pas l'injection)

**ARM-feedback (2 xfail)**
- `efficacy_arm_reads_d_fb_det_1`, `efficacy_arm_reads_a_cd` — l'ARM ne re-lit pas ce qu'on injecte (intentionnellement xfail, c'est le test du mur)

**L3 MM cascade entière (5 xfail)**
- `rach_emitted`, `immediate_assignment_decoded`, `rr_sdcch_established`, `location_updating_request_sent`, `location_updating_accept_received`
- → tout dépend de DATA_IND, donc xfail propres

**Sources d'init AR3/AR4 (2 skip)**
- Candidats littéraux trouvés mais pas confirmés (8 sites lk=0x0000, 18 sites lk≈0x2bc0)

**BSP DMA (1 xfail)**
- `bridge_fn_drift_under_threshold` (runtime)
- `d_fb_det_pattern_unchanged` (DSP runtime)
- `rach_attempted`, `neigh_pm_req_response_unchanged` (L1-ctrl)

## Lecture rapide

Tout ce qui touche **"observer / injecter / lire des states"** marche. Tout ce qui touche **"propager un événement temporel correctement"** ne marche pas. Le système est **dataflow-complet, time-flow-cassé**.

Le pivot conceptuel pour la suite : tu peux écrire, tu peux lire, tu peux observer — mais tu ne peux pas **synchroniser**. C'est exactement le bon angle pour passer au wall-clock domain.

# Features et état OK/NOK — groupé par `marker`

**Compteurs globaux :**
`passed=113 · failed=10 · skipped=3 · xfailed=23`

## 🔴 `drift` — 6/10 OK (4 NOK, 0 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| NOK | `test_bridge_qfn_advances_steadily` | `drift` | `Drift` | `Timing` | 0.26s | test_layer_drift.py:160: in test_bridge_qfn_advances_steadily |
| NOK | `test_bridge_qfn_tracks_qemu_fn` | `drift` | `Drift` | `Timing` | 0.19s | test_layer_drift.py:131: in test_bridge_qfn_tracks_qemu_fn |
| NOK | `test_log_start_within_10s` | `drift` | `Drift` | `Timing` | 0.47s | test_layer_drift.py:200: in test_log_start_within_10s |
| NOK | `test_qemu_insn_rate_cv_below_0_4` | `drift` | `Drift` | `Timing` | 0.11s | test_layer_drift.py:93: in test_qemu_insn_rate_cv_below_0_4 |
| OK | `test_log_still_growing[bridge-/tmp/bridge.log-10]` | `drift,parametrize` | `Drift` | `Timing` | 0.08s |  |
| OK | `test_log_still_growing[osmocon-/tmp/osmocon.log-1]` | `drift,parametrize` | `Drift` | `Timing` | 0.17s |  |
| OK | `test_log_still_growing[qemu-/root/qemu.log-1000]` | `drift,parametrize` | `Drift` | `Timing` | 0.14s |  |
| OK | `test_no_long_gap[bridge-/tmp/bridge.log-10.0]` | `drift,parametrize` | `Drift` | `Timing` | 0.07s |  |
| OK | `test_no_long_gap[qemu-/root/qemu.log-5.0]` | `drift,parametrize` | `Drift` | `Timing` | 0.07s |  |
| OK | `test_qemu_insn_rate_p1_above_1m` | `drift` | `Drift` | `Timing` | 0.08s |  |

## ⚪ `inject_efficacy` — 0/2 OK (0 NOK, 2 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| XFAIL | `test_efficacy_arm_reads_a_cd` | `inject_efficacy,inject_frames` | `ARM-feedback` | `Injection` | 3.07s | test_inject_frames.py:315: in test_efficacy_arm_reads_a_cd |
| XFAIL | `test_efficacy_arm_reads_d_fb_det_1` | `inject_efficacy,inject_frames` | `ARM-feedback` | `Injection` | 3.16s | test_inject_frames.py:290: in test_efficacy_arm_reads_d_fb_det_1 |

## ✅ `inject_frames` — 19/19 OK (0 NOK, 0 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_clear_ndb` | `inject_frames` | `NDB` | `Injection` | 0.34s |  |
| OK | `test_inject_a_cd_invalid_length` | `inject_frames` | `NDB` | `Injection` | 0.00s |  |
| OK | `test_inject_a_cd_pattern_23B` | `inject_frames` | `NDB` | `Injection` | 0.37s |  |
| OK | `test_inject_a_cd_pattern_30B` | `inject_frames` | `NDB` | `Injection` | 0.33s |  |
| OK | `test_inject_d_burst[0]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.08s |  |
| OK | `test_inject_d_burst[1]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.04s |  |
| OK | `test_inject_d_task[24-0]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.08s |  |
| OK | `test_inject_d_task[28-1]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.04s |  |
| OK | `test_inject_d_task[5-0]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.04s |  |
| OK | `test_inject_d_task[6-1]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.04s |  |
| OK | `test_inject_fbsb_fb_found` | `inject_frames` | `NDB` | `Injection` | 0.66s |  |
| OK | `test_inject_fbsb_sb_found` | `inject_frames` | `NDB` | `Injection` | 0.58s |  |
| OK | `test_inject_si[1]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.33s |  |
| OK | `test_inject_si[2]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.33s |  |
| OK | `test_inject_si[3]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.33s |  |
| OK | `test_inject_si[4]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.37s |  |
| OK | `test_inject_si[5]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.33s |  |
| OK | `test_inject_si[6]` | `inject_frames,parametrize` | `NDB` | `Injection` | 0.37s |  |
| OK | `test_probe_ndb` | `inject_frames` | `NDB` | `Injection` | 0.29s |  |

## 🟡 `milestone_bsp_dma` — 2/4 OK (0 NOK, 0 XFAIL, 2 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_bsp_dma_target_matches_correlator_read_zone` | `milestone_bsp_dma` | `L1-data` | `Milestone` | 0.14s |  |
| OK | `test_no_d_fb_det_wr_site_anomaly` | `milestone_bsp_dma` | `L1-data` | `Milestone` | 0.16s |  |
| SKIP | `test_ar3_init_source_identified` | `milestone_bsp_dma` | `L1-data` | `Milestone` | 0.01s | ('/home/nirvana/qemu-calypso/tests/test_calypso_milestones.py', 368, "Skipped: Source littérale (i) candidate : 8 sites  |
| SKIP | `test_ar4_init_source_identified` | `milestone_bsp_dma` | `L1-data` | `Milestone` | 0.01s | ('/home/nirvana/qemu-calypso/tests/test_calypso_milestones.py', 387, "Skipped: Source littérale (i) candidate AR4 : 18 s |

## 🟡 `milestone_dsp_decoder` — 4/5 OK (0 NOK, 1 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_dsp_throughput_5x` | `milestone_dsp_decoder` | `DSP` | `Milestone` | 0.16s |  |
| OK | `test_intm_dwell_no_regression` | `milestone_dsp_decoder` | `DSP` | `Milestone` | 30.41s |  |
| OK | `test_popm_decoder_active` | `milestone_dsp_decoder` | `DSP` | `Milestone` | 0.00s |  |
| OK | `test_tier_a_decoder_fixes_present` | `milestone_dsp_decoder` | `DSP` | `Milestone` | 0.00s |  |
| XFAIL | `test_rxdoneflag_no_longer_blocks` | `milestone_dsp_decoder` | `DSP` | `Milestone` | 0.00s | test_calypso_milestones.py:638: in test_rxdoneflag_no_longer_blocks |

## ⚪ `milestone_fb_det` — 0/2 OK (0 NOK, 1 XFAIL, 1 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| SKIP | `test_synth_zero_path_active` | `milestone_fb_det` | `DSP` | `Milestone` | 0.16s | ('/home/nirvana/qemu-calypso/tests/test_calypso_milestones.py', 506, 'Skipped: CALYPSO_FBSB_SYNTH=1 actif côté container |
| XFAIL | `test_fb0_att_nonzero` | `milestone_fb_det` | `DSP` | `Milestone` | 0.00s | test_calypso_milestones.py:474: in test_fb0_att_nonzero |

## ⚪ `milestone_irq` — 0/3 OK (0 NOK, 3 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| XFAIL | `test_c54x_interrupt_ex_called_nonzero` | `milestone_irq` | `DSP` | `Milestone` | 0.00s | test_calypso_milestones.py:527: in test_c54x_interrupt_ex_called_nonzero |
| XFAIL | `test_isr_entered_implies_rete` | `milestone_irq` | `DSP` | `Milestone` | 0.00s | test_calypso_milestones.py:536: in test_isr_entered_implies_rete |
| XFAIL | `test_no_pending_irq_gated` | `milestone_irq` | `DSP` | `Milestone` | 0.00s | test_calypso_milestones.py:545: in test_no_pending_irq_gated |

## 🔴 `milestone_l1ctl` — 0/2 OK (1 NOK, 1 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| NOK | `test_l1ctl_data_ind_received` | `milestone_l1ctl` | `L1-ctrl` | `Milestone` | 0.38s | test_calypso_milestones.py:582: in test_l1ctl_data_ind_received |
| XFAIL | `test_neigh_pm_req_response_unchanged` | `milestone_l1ctl` | `L1-ctrl` | `Milestone` | 0.00s | test_calypso_milestones.py:594: in test_neigh_pm_req_response_unchanged |

## ⚪ `milestone_mm_lu` — 0/5 OK (0 NOK, 5 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| XFAIL | `test_immediate_assignment_decoded` | `milestone_mm_lu` | `L3-MM` | `Milestone` | 0.00s | test_calypso_milestones.py:608: in test_immediate_assignment_decoded |
| XFAIL | `test_location_updating_accept_received` | `milestone_mm_lu` | `L3-MM` | `Milestone` | 0.00s | test_calypso_milestones.py:624: in test_location_updating_accept_received |
| XFAIL | `test_location_updating_request_sent` | `milestone_mm_lu` | `L3-MM` | `Milestone` | 0.00s | test_calypso_milestones.py:618: in test_location_updating_request_sent |
| XFAIL | `test_rach_emitted` | `milestone_mm_lu` | `L3-MM` | `Milestone` | 0.00s | test_calypso_milestones.py:603: in test_rach_emitted |
| XFAIL | `test_rr_sdcch_established` | `milestone_mm_lu` | `L3-MM` | `Milestone` | 0.00s | test_calypso_milestones.py:613: in test_rr_sdcch_established |

## 🟡 `runtime_bridge` — 2/3 OK (0 NOK, 1 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_bridge_dl_lookahead_respected` | `runtime_bridge` | `L1-data` | `Runtime` | 0.13s |  |
| OK | `test_bridge_log_shows_traffic` | `runtime_bridge` | `L1-data` | `Runtime` | 0.06s |  |
| XFAIL | `test_bridge_fn_drift_under_threshold` | `runtime_bridge` | `L1-data` | `Runtime` | 0.00s | test_run_observability.py:638: in test_bridge_fn_drift_under_threshold |

## 🟡 `runtime_dsp` — 6/7 OK (0 NOK, 1 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_bsp_daram_write_distribution` | `runtime_dsp` | `DSP` | `Runtime` | 0.17s |  |
| OK | `test_d_fb_det_data_no_longer_zero` | `runtime_dsp` | `DSP` | `Runtime` | 0.29s |  |
| OK | `test_dsp_throughput_above_threshold` | `runtime_dsp` | `DSP` | `Runtime` | 0.14s |  |
| OK | `test_intm_reaches_zero` | `runtime_dsp` | `DSP` | `Runtime` | 0.20s |  |
| OK | `test_no_enter_7740_dwell` | `runtime_dsp` | `DSP` | `Runtime` | 0.41s |  |
| OK | `test_no_wait_a21a_on_window` | `runtime_dsp` | `DSP` | `Runtime` | 0.29s |  |
| XFAIL | `test_d_fb_det_pattern_unchanged` | `runtime_dsp` | `DSP` | `Runtime` | 0.17s | test_run_observability.py:492: in test_d_fb_det_pattern_unchanged |

## ✅ `runtime_fs` — 3/3 OK (0 NOK, 0 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_container_disk_space_above_min` | `runtime_fs` | `FS` | `Runtime` | 0.06s |  |
| OK | `test_qemu_fd_usage_below_limit` | `runtime_fs` | `FS` | `Runtime` | 0.06s |  |
| OK | `test_qemu_log_disk_size_under_2gb` | `runtime_fs` | `FS` | `Runtime` | 0.05s |  |

## ✅ `runtime_gdb` — 8/8 OK (0 NOK, 0 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_gdb_handshake_succeeds` | `runtime_gdb` | `GDB-introspect` | `Runtime` | 0.54s |  |
| OK | `test_gdb_query_supported` | `runtime_gdb` | `GDB-introspect` | `Runtime` | 0.00s |  |
| OK | `test_gdb_read_arm_registers` | `runtime_gdb` | `GDB-introspect` | `Runtime` | 0.00s |  |
| OK | `test_gdb_read_dsp_daram_xfail` | `runtime_gdb` | `GDB-introspect` | `Runtime` | 0.00s |  |
| OK | `test_gdb_read_memory_at_dsp_api_base` | `runtime_gdb` | `GDB-introspect` | `Runtime` | 0.00s |  |
| OK | `test_gdb_read_pc_nonzero` | `runtime_gdb` | `GDB-introspect` | `Runtime` | 0.00s |  |
| OK | `test_gdb_stub_reachable` | `runtime_gdb` | `GDB-introspect` | `Runtime` | 0.00s |  |
| OK | `test_gdb_stub_survives_3_quick_reconnects` | `runtime_gdb` | `GDB-introspect` | `Runtime` | 0.46s |  |

## ✅ `runtime_health` — 5/5 OK (0 NOK, 0 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_all_expected_processes_present` | `runtime_health` | `Infra` | `Runtime` | 0.14s |  |
| OK | `test_mobile_pcap_growing` | `runtime_health` | `Infra` | `Runtime` | 10.00s |  |
| OK | `test_no_zombie_or_defunct` | `runtime_health` | `Infra` | `Runtime` | 0.09s |  |
| OK | `test_qemu_log_is_fresh` | `runtime_health` | `Infra` | `Runtime` | 3.10s |  |
| OK | `test_volumes_mounted` | `runtime_health` | `Infra` | `Runtime` | 0.15s |  |

## 🟡 `runtime_irda` — 7/8 OK (0 NOK, 1 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_irda_capture_process_alive` | `runtime_irda` | `IrDA-channel` | `Runtime` | 0.37s |  |
| OK | `test_irda_channel_produces_bytes` | `runtime_irda` | `IrDA-channel` | `Runtime` | 5.18s |  |
| OK | `test_irda_does_not_break_uart_modem` | `runtime_irda` | `IrDA-channel` | `Runtime` | 0.15s |  |
| OK | `test_irda_log_has_timestamp_prefix` | `runtime_irda` | `IrDA-channel` | `Runtime` | 0.20s |  |
| OK | `test_irda_pty_exists` | `runtime_irda` | `IrDA-channel` | `Runtime` | 0.13s |  |
| OK | `test_irda_pty_readable` | `runtime_irda` | `IrDA-channel` | `Runtime` | 0.42s |  |
| OK | `test_irda_throughput_below_saturation` | `runtime_irda` | `IrDA-channel` | `Runtime` | 10.24s |  |
| XFAIL | `test_irda_boot_marker_present` | `runtime_irda` | `IrDA-channel` | `Runtime` | 0.38s | test_irda_channel.py:140: in test_irda_boot_marker_present |

## ⚪ `runtime_irq` — 0/3 OK (0 NOK, 3 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| XFAIL | `test_interrupt_ex_called_counter_exposed` | `runtime_irq` | `DSP` | `Runtime` | 0.06s | test_run_observability.py:898: in test_interrupt_ex_called_counter_exposed |
| XFAIL | `test_isr_entered_matches_rete` | `runtime_irq` | `DSP` | `Runtime` | 0.06s | test_run_observability.py:916: in test_isr_entered_matches_rete |
| XFAIL | `test_no_pending_irq_gating` | `runtime_irq` | `DSP` | `Runtime` | 0.08s | test_run_observability.py:928: in test_no_pending_irq_gating |

## 🔴 `runtime_l1ctl` — 3/6 OK (2 NOK, 1 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| NOK | `test_l1ctl_data_ind_rate_vs_alc` | `runtime_l1ctl` | `L1-ctrl` | `Runtime` | 0.58s | test_run_observability.py:792: in test_l1ctl_data_ind_rate_vs_alc |
| NOK | `test_l1ctl_data_ind_received` | `runtime_l1ctl` | `L1-ctrl` | `Runtime` | 0.39s | test_run_observability.py:718: in test_l1ctl_data_ind_received |
| OK | `test_a_cd_write_pc_includes_ccch_demod` | `runtime_l1ctl` | `L1-ctrl` | `Runtime` | 0.51s |  |
| OK | `test_a_cd_writes_nonzero` | `runtime_l1ctl` | `L1-ctrl` | `Runtime` | 0.30s |  |
| OK | `test_neigh_pm_req_loop_alive` | `runtime_l1ctl` | `L1-ctrl` | `Runtime` | 0.77s |  |
| XFAIL | `test_rach_attempted` | `runtime_l1ctl` | `L1-ctrl` | `Runtime` | 0.00s | test_run_observability.py:797: in test_rach_attempted |

## 🔴 `runtime_log_grep` — 27/30 OK (1 NOK, 2 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| NOK | `test_grep_qemu_dsp_booted` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.07s | test_log_grep.py:77: in test_grep_qemu_dsp_booted |
| OK | `test_blocker_bridge_no_bts_shutdown` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.21s |  |
| OK | `test_blocker_bridge_no_rach_parity` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.13s |  |
| OK | `test_blocker_bridge_no_socket_error` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.12s |  |
| OK | `test_blocker_fwirda_no_panic` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.15s |  |
| OK | `test_blocker_mobile_no_crash` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.11s |  |
| OK | `test_blocker_mobile_no_vty_bind_error` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.13s |  |
| OK | `test_blocker_no_out_of_memory` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.07s |  |
| OK | `test_blocker_osmocon_no_connection_refused` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.11s |  |
| OK | `test_blocker_osmocon_no_layer2_socket_failed` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.21s |  |
| OK | `test_blocker_osmocon_no_pty_error` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.15s |  |
| OK | `test_blocker_qemu_no_assert_failed` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.07s |  |
| OK | `test_blocker_qemu_no_long_wait_a21a` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.09s |  |
| OK | `test_blocker_qemu_no_panic` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.07s |  |
| OK | `test_blocker_qemu_no_qemu_abort` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.06s |  |
| OK | `test_blocker_qemu_no_runaway_dsp` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.06s |  |
| OK | `test_grep_bridge_dl_bursts_received` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.12s |  |
| OK | `test_grep_bridge_fb_pattern_dominant` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.18s |  |
| OK | `test_grep_bridge_no_clock_skew_shutdown` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.14s |  |
| OK | `test_grep_fwirda_log_present_or_skip` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.05s |  |
| OK | `test_grep_mobile_alive_signal` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.12s |  |
| OK | `test_grep_osmocon_l1ctl_fbsb_req_attempted` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.12s |  |
| OK | `test_grep_osmocon_l1ctl_reset_req` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.11s |  |
| OK | `test_grep_osmocon_lost_ratio_acceptable` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.18s |  |
| OK | `test_grep_qemu_bsp_dma_active` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.15s |  |
| OK | `test_grep_qemu_log_exists_and_nonempty` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.12s |  |
| OK | `test_grep_qemu_no_sp_catastrophe_recent` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.06s |  |
| OK | `test_grep_qemu_task24_dispatched` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.09s |  |
| XFAIL | `test_grep_fwirda_boot_marker` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.10s | test_log_grep.py:203: in test_grep_fwirda_boot_marker |
| XFAIL | `test_grep_qemu_a_cd_wr_vs_task24` | `runtime_log_grep` | `Log-grep` | `Runtime` | 0.22s | test_log_grep.py:362: in test_grep_qemu_a_cd_wr_vs_task24 |

## ✅ `runtime_monitor` — 9/9 OK (0 NOK, 0 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_monitor_info_chardev_lists_serial` | `runtime_monitor` | `Monitor-extended` | `Runtime` | 0.07s |  |
| OK | `test_monitor_info_irq_listed` | `runtime_monitor` | `Monitor-extended` | `Runtime` | 0.18s |  |
| OK | `test_monitor_info_mtree_has_uart_irda` | `runtime_monitor` | `Monitor-extended` | `Runtime` | 0.14s |  |
| OK | `test_monitor_info_mtree_has_uart_modem` | `runtime_monitor` | `Monitor-extended` | `Runtime` | 0.08s |  |
| OK | `test_monitor_info_qom_tree_has_calypso_or_arm` | `runtime_monitor` | `Monitor-extended` | `Runtime` | 0.20s |  |
| OK | `test_monitor_info_qtree_has_calypso` | `runtime_monitor` | `Monitor-extended` | `Runtime` | 0.06s |  |
| OK | `test_monitor_info_registers_arm` | `runtime_monitor` | `Monitor-extended` | `Runtime` | 0.14s |  |
| OK | `test_monitor_info_status_is_running` | `runtime_monitor` | `Monitor-extended` | `Runtime` | 0.09s |  |
| OK | `test_monitor_socket_reachable` | `runtime_monitor` | `Monitor-extended` | `Runtime` | 0.11s |  |

## ✅ `runtime_net` — 2/2 OK (0 NOK, 0 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_no_unexpected_high_ports` | `runtime_net` | `Net` | `Runtime` | 0.07s |  |
| OK | `test_qemu_ports_listening` | `runtime_net` | `Net` | `Runtime` | 0.06s |  |

## ✅ `runtime_summary` — 1/1 OK (0 NOK, 0 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_run_summary_snapshot` | `runtime_summary` | `Summary` | `Runtime` | 31.09s |  |

## ✅ `runtime_vty` — 3/3 OK (0 NOK, 0 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| OK | `test_mobile_imsi_loaded` | `runtime_vty` | `Mgmt` | `Runtime` | 1.61s |  |
| OK | `test_mobile_mm_state_is_null_or_idle` | `runtime_vty` | `Mgmt` | `Runtime` | 1.64s |  |
| OK | `test_mobile_vty_reachable` | `runtime_vty` | `Mgmt` | `Runtime` | 1.62s |  |

## 🔴 `timer_invariant` — 6/9 OK (2 NOK, 1 XFAIL, 0 SKIP)

| État | Test | Marker | Couche | Catégorie | Durée |  Assertion (si NOK)  |
|---|---|---|---|---|---:|---|
| NOK | `test_kick_realtime_cadence` | `timer_invariant` | `Timers` | `Timing` | 0.00s | test_timer_invariants.py:197: in test_kick_realtime_cadence |
| NOK | `test_tdma_period_virtual_close_to_target` | `timer_invariant` | `Timers` | `Timing` | 0.00s | test_timer_invariants.py:167: in test_tdma_period_virtual_close_to_target |
| OK | `test_dsp_n_exec_within_budget` | `timer_invariant` | `Timers` | `Timing` | 0.00s |  |
| OK | `test_frame_irq_per_tdma_ratio` | `timer_invariant` | `Timers` | `Timing` | 0.00s |  |
| OK | `test_log_timeline_csv_no_dead_bucket` | `timer_invariant` | `Timers` | `Timing` | 0.06s |  |
| OK | `test_log_timeline_csv_produced` | `timer_invariant` | `Timers` | `Timing` | 0.06s |  |
| OK | `test_log_timeline_csv_steady_qemu_rate` | `timer_invariant` | `Timers` | `Timing` | 0.07s |  |
| OK | `test_tdma_log_present` | `timer_invariant` | `Timers` | `Timing` | 0.00s |  |
| XFAIL | `test_dsp_budget_saturation_signal` | `timer_invariant` | `Timers` | `Timing` | 0.00s | test_timer_invariants.py:148: in test_dsp_budget_saturation_signal |
