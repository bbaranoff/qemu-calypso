# ============================================================================
# calypso_l1_trace.gdb — trace le chemin ARM L1 ↔ DSP vers le 28868
# ----------------------------------------------------------------------------
# But : répondre "ça diverge à la place de c54x ?" — vérifier que l'ARM
# progresse NORMALEMENT (FBSB → FB det → SB det → NB) et voir quelles tâches
# il poste au DSP. Si l'ARM tourne sain et poste les bonnes tâches → le 28868
# est purement DSP (c54x). Si l'ARM se fige/diverge aussi → autre front.
#
# Usage (dans le conteneur osmo-operator-1, QEMU déjà lancé avec -gdb tcp::1234) :
#   gdb-multiarch /opt/GSM/firmware/board/compal_e88/layer1.highram.elf \
#       -x /opt/GSM/qemu-src/calypso_l1_trace.gdb
# ============================================================================

set pagination off
set confirm off
set breakpoint pending on
set architecture arm

# Connexion : seulement si PAS déjà attaché. Si tu `source` ce fichier dans un
# gdb déjà connecté (pane run.sh), laisse la ligne ci-dessous COMMENTÉE.
# Pour un gdb lancé à la main non connecté, décommente-la :
# target remote :1234

# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------

# Dump des mots de contrôle API RAM vus côté ARM (base 0xFFD00000, byte offsets
# per CLAUDE.md). Montre quelle tâche DSP l'ARM a posée ce frame.
define apidump
  printf "  [API] d_task_d=0x%04x d_burst_d=0x%04x d_task_u=0x%04x d_burst_u=0x%04x d_task_md=0x%04x d_task_ra=0x%04x\n", \
    *(unsigned short *)0xFFD00000, *(unsigned short *)0xFFD00002, \
    *(unsigned short *)0xFFD00004, *(unsigned short *)0xFFD00006, \
    *(unsigned short *)0xFFD00008, *(unsigned short *)0xFFD0000E
  printf "  [NDB] d_fb_det=0x%04x  d_error_status=0x%04x\n", \
    *(unsigned short *)0xFFD001F0, *(unsigned short *)0xFFD001AA
end

# Contexte ARM rapide
define ctx
  printf "  PC=0x%08x LR=0x%08x SP=0x%08x  r0=0x%x r1=0x%x r2=0x%x\n", \
    $pc, $lr, $sp, $r0, $r1, $r2
end

# ----------------------------------------------------------------------------
# Tracepoints SYNC (FBSB) — silencieux, printf + continue.
# Ils logent le flux sans arrêter. Commente `continue` pour t'arrêter dessus.
# ----------------------------------------------------------------------------

break l1s_fbsb_req
commands
  silent
  printf ">> l1s_fbsb_req           (demande acquisition sync)\n"
  apidump
  continue
end

break l1s_fbdet_cmd
commands
  silent
  printf ">> l1s_fbdet_cmd          (poste tâche FB-det au DSP)\n"
  apidump
  continue
end

break l1s_fbdet_resp
commands
  silent
  printf ">> l1s_fbdet_resp         (lit résultat FB-det du DSP) d_fb_det suit:\n"
  apidump
  continue
end

break l1s_sbdet_cmd
commands
  silent
  printf ">> l1s_sbdet_cmd          (poste tâche SB-det)\n"
  continue
end

break l1s_sbdet_resp
commands
  silent
  printf ">> l1s_sbdet_resp         (lit résultat SB-det)\n"
  apidump
  continue
end

break l1ctl_fbsb_resp
commands
  silent
  printf ">> l1ctl_fbsb_resp        (FBSB ABOUTI → notifie l'host L2 !)\n"
  ctx
  continue
end

# ----------------------------------------------------------------------------
# Tracepoints DSP-API bridge — ce que l'ARM commande au DSP.
# ----------------------------------------------------------------------------

break dsp_load_rx_task
commands
  silent
  printf ">> dsp_load_rx_task  task=0x%04x burst=0x%x  (RX→DSP)\n", $r0, $r1
  continue
end

break dsp_load_tx_task
commands
  silent
  printf ">> dsp_load_tx_task  task=0x%04x  (TX→DSP)\n", $r0
  continue
end

break dsp_end_scenario
commands
  silent
  printf ">> dsp_end_scenario       (commit frame DSP)\n"
  continue
end

# ----------------------------------------------------------------------------
# STOP utile : la 1ère acquisition. Décommente pour t'arrêter et inspecter.
# ----------------------------------------------------------------------------
# tbreak l1s_fbsb_req
# commands
#   printf "=== STOP sur 1er l1s_fbsb_req — inspecte avec: ctx / apidump / bt ===\n"
# end

printf "[gdb] tracepoints L1↔DSP armés. `continue` pour lancer le flux.\n"
printf "[gdb] helpers: apidump (mots API), ctx (registres). Ctrl-C pour reprendre la main.\n"
