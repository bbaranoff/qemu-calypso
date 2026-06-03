docker exec osmo-operator-1 python3 -c "
import io
# cfg 1 SPS
c='/opt/GSM/qemu-src/cfgs/osmo-trx-ipc.cfg'; s=io.open(c).read()
io.open(c,'w').write(s.replace('tx-sps 4','tx-sps 1').replace('rx-sps 4','rx-sps 1'))
# device OSR=1 (BUFSIZE 625, burst 148, frame 1250 redeviennent corrects)
q='/opt/GSM/qemu-src/tools/calypso-ipc-device/qemu_wrap.c'; t=io.open(q).read()
t=t.replace('#define CALYPSO_TRX_OSR       4','#define CALYPSO_TRX_OSR       1').replace('#define CALYPSO_SHM_BUFSIZE   2500','#define CALYPSO_SHM_BUFSIZE   625')
io.open(q,'w').write(t); print('revert 1 SPS OK')"
docker exec osmo-operator-1 bash -lc 'cd /opt/GSM/qemu-src/tools/calypso-ipc-device && make 2>&1|grep -i error; echo rc=$?'
