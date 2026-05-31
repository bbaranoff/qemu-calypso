docker exec -i osmo-operator-1 bash -c "cat > /tmp/mutate_agch.gdb" <<'EOF'
set $rotate = 0

# Payload IMMEDIATE ASSIGNMENT minimal (23 octets) :
# 2d 06 3f 0020 00 0100 00 0000 00 2b 2b 2b 2b 2b 2b 2b 2b 2b 2b 2b
# pseudo-len=11(L3), PD=RR, MT=IMM ASS, PM=0/DedMode=0
# ChanDesc=SDCCH/4 sub0 TS0 ARFCN=1, ReqRef RA=0 T=0, TA=0, MA len=0, rest octets padding

define mutate_agch
delete
break l1_queue_for_l2
commands
silent
set $hd = (struct l1ctl_hdr*)msg->l1h
if $hd->msg_type == 3
set $dl = (struct l1ctl_info_dl*)((char*)msg->l1h + sizeof(struct l1ctl_hdr))
set $b= (uint8_t*)$dl + sizeof(*$dl)

# === BCCH (0x80) : continue à roter SI1/SI2/SI3/SI4 ===
if $dl->chan_nr == 0x80
if $rotate == 0
set $b[2] = 0x19
end
if $rotate == 1
set $b[2] = 0x1a
end
if $rotate == 2
set $b[2] = 0x1b
end
if $rotate == 3
set $b[2] = 0x1c
end
set $rotate = ($rotate + 1) % 4
printf "[bcch] L3=%02x\n", $b[2]
end

# === PCH/AGCH (0x90) : injecte IMM ASSIGNMENT ===
if $dl->chan_nr == 0x90
set $b[0]= 0x2d
set $b[1]= 0x06
set $b[2]= 0x3f
set $b[3]= 0x00
set $b[4]= 0x20
set $b[5]= 0x00
set $b[6]= 0x01
set $b[7]= 0x00
set $b[8]= 0x00
set $b[9]= 0x00
set $b[10] = 0x00
set $b[11] = 0x00
set $b[12] = 0x2b
set $b[13] = 0x2b
set $b[14] = 0x2b
set $b[15] = 0x2b
set $b[16] = 0x2b
set $b[17] = 0x2b
set $b[18] = 0x2b
set $b[19] = 0x2b
set $b[20] = 0x2b
set $b[21] = 0x2b
set $b[22] = 0x2b
printf "[agch] IMM ASS injected fn=%u\n", $dl->frame_nr
end
end
c
end
end

# Variante : n'injecte IMM ASS qu'aux ~10% des AGCH (pour pas flood)
set $agch_cnt = 0
define mutate_agch_thin
delete
break l1_queue_for_l2
commands
silent
set $hd = (struct l1ctl_hdr*)msg->l1h
if $hd->msg_type == 3
set $dl = (struct l1ctl_info_dl*)((char*)msg->l1h + sizeof(struct l1ctl_hdr))
set $b= (uint8_t*)$dl + sizeof(*$dl)
if $dl->chan_nr == 0x80
if $rotate == 0
set $b[2] = 0x19
end
if $rotate == 1
set $b[2] = 0x1a
end
if $rotate == 2
set $b[2] = 0x1b
end
if $rotate == 3
set $b[2] = 0x1c
end
set $rotate = ($rotate + 1) % 4
end
if $dl->chan_nr == 0x90
set $agch_cnt = $agch_cnt + 1
if ($agch_cnt % 10) == 0
set $b[0]=0x2d
set $b[1]=0x06
set $b[2]=0x3f
set $b[3]=0x00
set $b[4]=0x20
set $b[5]=0x00
set $b[6]=0x01
set $b[7]=0x00
set $b[8]=0x00
set $b[9]=0x00
set $b[10]=0x00
set $b[11]=0x00
set $b[12]=0x2b
set $b[13]=0x2b
set $b[14]=0x2b
set $b[15]=0x2b
set $b[16]=0x2b
set $b[17]=0x2b
set $b[18]=0x2b
set $b[19]=0x2b
set $b[20]=0x2b
set $b[21]=0x2b
set $b[22]=0x2b
printf "[agch %d] IMM ASS injected fn=%u\n", $agch_cnt, $dl->frame_nr
end
end
end
c
end
end
EOF
