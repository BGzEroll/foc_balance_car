"""Stopped-board OTA acceptance suite. Does not send CAL, BALANCE or PROBE."""
import hashlib
import http.client
import json
from pathlib import Path
import socket
import time
from client import OTA, ROOT, HERE, DebugClient, image_info, stopped


def run(host='192.168.114.243'):
    ota=OTA(host)
    image=ROOT/'esp32/wifi_debug/firmware/balance_car.bin'
    data=image.read_bytes(); info=image_info(data)
    evidence={'initial':ota.request('status'), 'stopped_before':stopped(host), 'image':info}
    def save():
        (HERE/'results').mkdir(exist_ok=True)
        (HERE/'results/acceptance.json').write_text(json.dumps(evidence,indent=2))
    def reject(body, sha, code, token=None, schema='current_pid_v1'):
        connection=http.client.HTTPConnection(host,3335,timeout=15)
        try:
            connection.request('POST','/ota/upload',body,headers={'X-OTA-Token':token or ota.token,
                'X-Image-SHA256':sha,'X-Image-Schema':schema,'Connection':'close'})
            response=connection.getresponse();result=json.loads(response.read())
            assert response.status==code,(response.status,result)
            return dict(status=response.status,response=result)
        finally: connection.close()
    def unchanged(previous):
        status=ota.request('status')
        assert status['running']==previous['running'] and status['boot']==previous['boot']
        assert not status['locked'] and not status['staged'] and not status['pending_verify']
        stopped(host)
        return status
    baseline=ota.request('status')
    assert not baseline['pending_verify'] and not baseline['locked']
    evidence['unauthorized']=reject(b'x'*1024,'0'*64,401,token='0'*64)
    evidence['wrong_schema']=reject(b'x'*1024,'0'*64,400,schema='torque_v0')
    sock=socket.create_connection((host,3335),timeout=5)
    headers=(f'POST /ota/upload HTTP/1.1\r\nHost: {host}\r\nX-OTA-Token: {ota.token}\r\n'
        f'X-Image-SHA256: {info["sha256"]}\r\nX-Image-Schema: current_pid_v1\r\n'
        f'Content-Length: {len(data)}\r\nConnection: close\r\n\r\n').encode()
    sock.sendall(headers+data[:65536]);sock.close()
    time.sleep(4)
    evidence['interrupted_64k']=unchanged(baseline);save()
    print('PASS unauthorized, schema, interrupted upload',flush=True)
    evidence['wrong_hash']=reject(data,'0'*64,422)
    evidence['after_wrong_hash']=unchanged(baseline)
    corrupt=bytearray(data);corrupt[0x1000]^=1
    evidence['corrupt_image']=reject(corrupt,hashlib.sha256(corrupt).hexdigest(),422)
    evidence['after_corrupt']=unchanged(baseline);save()
    print('PASS hash mismatch and official image validation',flush=True)
    evidence['first_update']=ota.update(image);save()
    print('PASS real Wi-Fi update and explicit healthy boot confirmation',flush=True)
    evidence['cancel_stage']=ota.stage(image)
    c=DebugClient(host)
    try:
        try:
            c.command('SET pitch_trim_rad 0')
            raise AssertionError('Maintenance guard did not reject parameter mutation')
        except RuntimeError as exc:
            assert str(exc)=='REJECT OTA maintenance lock'
            evidence['maintenance_rejection']=str(exc)
        evidence['maintenance_status']=c.command('STATUS')
    finally: c.close()
    ota.request('cancel')
    baseline=ota.request('status')
    assert not baseline['locked'] and not baseline['staged']
    evidence['cancelled']=baseline;save()
    evidence['rollback_stage']=ota.stage(image)
    ota.request('reboot')
    evidence['rollback_pending']=ota.wait_boot(info['elf_sha256'],baseline['running'],confirm=False);save()
    print('PASS pending boot remains locked; waiting for automatic 60s rollback',flush=True)
    deadline=time.monotonic()+80
    while time.monotonic()<deadline:
        try:
            status=ota.request('status')
            if status['running']==baseline['running'] and not status['pending_verify'] and status['healthy']:
                assert not status['locked'] and status['state']==2
                evidence['rollback_recovered']=status;break
        except (OSError,http.client.HTTPException): pass
        time.sleep(2)
    else: raise TimeoutError('Automatic rollback not observed')
    save();print('PASS automatic rollback to previous valid image',flush=True)
    evidence['second_slot_update']=ota.update(image)
    evidence['stopped_after']=stopped(host)
    evidence['complete']=True;save()
    print('PASS both OTA slots and final healthy STOP',flush=True)


if __name__=='__main__': run()
