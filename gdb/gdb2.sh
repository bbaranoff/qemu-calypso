docker exec -i osmo-operator-1 bash -c "cat > /tmp/mutate_full.gdb" <<'EOF'
set $rotate = 0

define mutate_full
delete
break l1_queue_for_l2
commands
silent
set $hd = (struct l1ctl_hdr*)msg->l1h
if $hd->msg_type == 3
set $dl = (struct l1ctl_info_dl*)((char*)msg->l1h + sizeof(struct l1ctl_hdr))
set $b= (uint8_t*)$dl + sizeof(*$dl)
printf "[mf] ORIG chan=%02x type_L3=%02x -> ", $dl->chan_nr, $b[2]
set $dl->chan_nr = 0x80
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
printf "NEW chan=80 type_L3=%02x\n", $b[2]
end
c
end
end

# Variante : ne touche PAS chan_nr (laisse 0x80/0x90 originel), ne rotate que L3
define mutate_l3only
delete
break l1_queue_for_l2
commands
silent
set $hd = (struct l1ctl_hdr*)msg->l1h
if $hd->msg_type == 3
set $dl = (struct l1ctl_info_dl*)((char*)msg->l1h + sizeof(struct l1ctl_hdr))
set $b= (uint8_t*)$dl + sizeof(*$dl)
# ne rotate que sur les BCCH (chan=0x80), laisse PCH/AGCH tranquille
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
printf "[mf-bcch] chan=80 L3=%02x\n", $b[2]
end
end
c
end
end
EOF
