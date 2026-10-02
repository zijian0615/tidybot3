#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cert_dir="$script_dir/phone_https"
phone_ip="${1:-}"

if [[ -z "$phone_ip" ]]; then
  phone_ip="$(ip route get 1.1.1.1 | sed -n 's/.* src \([^ ]*\).*/\1/p' | head -n1)"
fi
if [[ -z "$phone_ip" ]]; then
  echo "Could not determine the Wi-Fi IP. Run: $0 10.x.x.x" >&2
  exit 1
fi

mkdir -p "$cert_dir"
chmod 700 "$cert_dir"

if [[ -f "$cert_dir/phone-ca.crt" ]] && \
   { ! openssl x509 -in "$cert_dir/phone-ca.crt" -noout -text | grep -q 'CA:TRUE' || \
     ! openssl x509 -in "$cert_dir/phone-ca.crt" -noout -text | grep -q 'Certificate Sign'; }; then
  echo "Replacing an older CA that lacks required CA extensions."
  rm -f "$cert_dir/phone-ca.crt" "$cert_dir/phone-ca.key" "$cert_dir/phone-ca.srl"
fi

if [[ ! -f "$cert_dir/phone-ca.key" ]]; then
  openssl req -x509 -newkey rsa:2048 -sha256 -nodes -days 3650 \
    -keyout "$cert_dir/phone-ca.key" -out "$cert_dir/phone-ca.crt" \
    -subj "/CN=TidyBot Phone CA" \
    -addext "basicConstraints=critical,CA:TRUE,pathlen:0" \
    -addext "keyUsage=critical,keyCertSign,cRLSign" \
    -addext "subjectKeyIdentifier=hash"
fi

cat >"$cert_dir/server.ext" <<EOF
subjectAltName=IP:$phone_ip,DNS:tidybot.local
extendedKeyUsage=serverAuth
keyUsage=digitalSignature,keyEncipherment
basicConstraints=critical,CA:FALSE
authorityKeyIdentifier=keyid,issuer
EOF
openssl req -newkey rsa:2048 -sha256 -nodes \
  -keyout "$cert_dir/server.key" -out "$cert_dir/server.csr" \
  -subj "/CN=$phone_ip"
openssl x509 -req -sha256 -days 365 -in "$cert_dir/server.csr" \
  -CA "$cert_dir/phone-ca.crt" -CAkey "$cert_dir/phone-ca.key" \
  -CAcreateserial -out "$cert_dir/server.crt" -extfile "$cert_dir/server.ext"
chmod 600 "$cert_dir"/*.key

cert_b64="$(openssl x509 -in "$cert_dir/phone-ca.crt" -outform DER | base64 -w 68)"
profile_uuid="$(cat /proc/sys/kernel/random/uuid)"
cert_uuid="$(cat /proc/sys/kernel/random/uuid)"
cat >"$cert_dir/tidybot-phone-ca.mobileconfig" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
<key>PayloadContent</key><array><dict>
<key>PayloadCertificateFileName</key><string>TidyBot Phone CA</string>
<key>PayloadContent</key><data>
$cert_b64
</data>
<key>PayloadDescription</key><string>Trusts the local TidyBot Safari teleoperation server.</string>
<key>PayloadDisplayName</key><string>TidyBot Phone CA</string>
<key>PayloadIdentifier</key><string>local.tidybot.phone.ca.certificate</string>
<key>PayloadType</key><string>com.apple.security.root</string>
<key>PayloadUUID</key><string>$cert_uuid</string>
<key>PayloadVersion</key><integer>1</integer>
</dict></array>
<key>PayloadDescription</key><string>Installs the local TidyBot root certificate.</string>
<key>PayloadDisplayName</key><string>TidyBot Phone CA</string>
<key>PayloadIdentifier</key><string>local.tidybot.phone.ca</string>
<key>PayloadOrganization</key><string>TidyBot</string>
<key>PayloadRemovalDisallowed</key><false/>
<key>PayloadType</key><string>Configuration</string>
<key>PayloadUUID</key><string>$profile_uuid</string>
<key>PayloadVersion</key><integer>1</integer>
</dict></plist>
EOF

cat <<EOF
Created an HTTPS certificate for $phone_ip.

1. Transfer phone_https/tidybot-phone-ca.mobileconfig to the iPhone and
   install the downloaded profile under Settings > General > VPN & Device
   Management.
2. In Settings > General > About > Certificate Trust Settings, enable full
   trust for "TidyBot Phone CA".
3. Start teleoperation with:
   python3 phone_base_teleop.py --tls-cert phone_https/server.crt \\
     --tls-key phone_https/server.key
4. Open https://$phone_ip:5000 in Safari.
EOF
