"""Encrypt the web workbench into one bundle for the password page on ascoding11.github.io.

    JD_PASSWORD=... python tools/build_vault.py <site>/joukowsky-dynamics/vault.bin

Bundle: salt(16) | iv(12) | AES-256-GCM( u32 header length | JSON {name: [offset, length]} | file bytes ).
Key: PBKDF2-HMAC-SHA256(lowercased password, salt, 310 000 iterations). The password is never written anywhere.
"""
import json, os, struct, sys
from cryptography.hazmat.primitives.ciphers.aead import AESGCM
from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
from cryptography.hazmat.primitives import hashes

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ITER = 310_000
FILES = ['index.html', 'base.css', 'style.css', 'revision.css', 'design.css', 'advanced/advanced.css',
         'app.js', 'advanced/advanced.js', 'advanced/data/parts.json', 'advanced/data/meshes.bin']

def site_tweaks(name, data):
    """The copy on the personal site gets a link back to the Engineering Projects page."""
    if name == 'index.html':
        data = data.replace(b'<div class="project-title">',
                            b'<div class="project-title"><a href="../engineering/" class="jd-back">&larr; Engineering projects</a>', 1)
    if name == 'advanced/advanced.css':
        data += (b'\n.project-title .jd-back { color: var(--accent); text-decoration: none; margin-right: 18px; }\n'
                 b'.project-title .jd-back:hover { text-decoration: underline; }\n')
    return data

def main(out):
    pw = os.environ.get('JD_PASSWORD', '')
    if not pw:
        sys.exit('set JD_PASSWORD')
    header, blobs, off = {}, [], 0
    for name in FILES:
        data = site_tweaks(name, open(os.path.join(ROOT, name), 'rb').read())
        header[name] = [off, len(data)]
        blobs.append(data); off += len(data)
    h = json.dumps(header).encode()
    payload = struct.pack('<I', len(h)) + h + b''.join(blobs)
    salt, iv = os.urandom(16), os.urandom(12)
    key = PBKDF2HMAC(algorithm=hashes.SHA256(), length=32, salt=salt, iterations=ITER).derive(pw.strip().lower().encode())
    ct = AESGCM(key).encrypt(iv, payload, None)
    with open(out, 'wb') as f:
        f.write(salt + iv + ct)
    print(f'wrote {out}: {len(FILES)} files, {len(payload) / 1e6:.2f} MB plaintext')

if __name__ == '__main__':
    main(sys.argv[1])
