docker exec -i osmo-operator-1 bash -c "cat > /tmp/efgh.gdb" <<'EOF'
set $n_q=0
set $n_create=0
set $n_nb=0
set $n_fbsb=0

# === f : tap sur l1_queue_for_l2 (TOUT msg firmware->host passe ici) ===
define f
break l1_queue_for_l2
commands
silent
set $n_q = $n_q + 1
set $hd = (struct l1ctl_hdr*)msg->l1h
printf "[f #%d] queue_for_l2 type=%02x len=%u", $n_q, $hd->msg_type, msg->len
if msg->len >= 23
set $b = (uint8_t*)msg->l1h + sizeof(struct l1ctl_hdr) + sizeof(struct l1ctl_info_dl)
printf " L3=%02x %02x %02x", $b[0], $b[1], $b[2]
end
printf "\n"
c
end
end

# === g : l1_create_l2_msg, juste avant remplissage ===
define g
break l1_create_l2_msg
commands
silent
set $n_create = $n_create + 1
printf "[g #%d] create_l2_msg type=%02x\n", $n_create, msg_type
c
end
end

# === h : tap sur l1s_nb_resp (arrivée d'un NB burst depuis DSP) ===
define h
break l1s_nb_resp
commands
silent
set $n_nb = $n_nb + 1
printf "[h #%d] nb_resp fn=%u\n", $n_nb, l1s.next_time.fn
c
end
end

# === bonus : tap sur FBSB pour confirmer sync cellulaire ===
define fbsb
break l1ctl_fbsb_resp
commands
silent
set $n_fbsb = $n_fbsb + 1
printf "[fbsb #%d] FBSB_RESP\n", $n_fbsb
c
end
end

define stats
printf "queue_for_l2=%d create=%d nb_resp=%d fbsb_resp=%d\n", \
$n_q, $n_create, $n_nb, $n_fbsb
end
EOF
