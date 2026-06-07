# GDB script for Calypso debug session (auto-loaded by run_debug.sh)
target remote :1234
set pagination off
set confirm off

# Convenience: print d_fb_det
define p_fb
    print/x dsp_api.ndb->d_fb_det
end

# Convenience: list FB-relevant NDB cells
define p_ndb
    printf "d_dsp_page=0x%04x  d_fb_det=0x%04x  d_fb_mode=0x%04x\n", \
        dsp_api.ndb->d_dsp_page, \
        dsp_api.ndb->d_fb_det,   \
        dsp_api.ndb->d_fb_mode
end

# Force "FB detected" path inside l1s_fbdet_resp.
# The compiler emits:
#   ldrh  r8, [r3, #72]   ; load d_fb_det
#   cmp   r8, #0
#   bne   <fb_found>
# We break right after the ldrh (PC = entry+0x10) and patch r8 = 1.
# Each hit prints the attempt number then continues silently.
break *0x826434
commands
silent
set $r8 = 1
printf "[FB-FORCE] r8=1 (was %d) attempt=%d fb_mode=%d\n", $r8, $r1, $r2
continue
end

# Pretty-print on stop
display/i $pc

echo \n[gdb] script loaded. Type 'continue' to start.\n
