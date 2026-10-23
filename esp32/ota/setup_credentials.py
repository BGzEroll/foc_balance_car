"""Create or reuse local OTA authorization. Keep token.json for future updates."""
import json
from pathlib import Path
import secrets

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
path=HERE/'token.json'
if not path.exists():
    path.write_text(json.dumps({'token':secrets.token_hex(32)}))
token=json.loads(path.read_text())['token']
assert len(token)==64 and all(c in '0123456789abcdef' for c in token)
(ROOT/'esp32/balance_car/user_lib/ota_credentials.h').write_text('#pragma once\n#define OTA_TOKEN "'+token+'"\n')
print('OTA authorization header ready; existing local token preserved; secrets not printed')
