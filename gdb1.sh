docker exec -i trying bash -c "cat > /tmp/mutate.gdb" <<'EOF'
set $rotate = 0

# Rotation SI3 -> SI4 -> SI1 -> SI2 -> SI13 -> SI3 ...
define mutate
delete
break l1_queue_for_l2
commands
silent
set $hd = (struct l1ctl_hdr*)msg->l1h
if $hd->msg_type == 3
set $b = (uint8_t*)msg->l1h + sizeof(struct l1ctl_hdr) + sizeof(struct l1ctl_info_dl)
# byte[2] = RR message type (SI*)
if $rotate == 0
set $b[2] = 0x19
end
if $rotate == 1
set $b[2] = 0x1a
end
if $rotate == 2
set $b[2] = 0x1c
end
if $rotate == 3
set $b[2] = 0x00
end
if $rotate == 4
set $b[2] = 0x1b
end
set $rotate = ($rotate + 1) % 5
printf "[mutate] -> type %02x\n", $b[2]
end
c
end
end
EOF
