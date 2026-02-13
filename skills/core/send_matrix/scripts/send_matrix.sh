#!/bin/sh

TOKEN=${MATRIX_TOKEN:-"DEIN_ACCESS_TOKEN"}
HOMESERVER=${MATRIX_HOMESERVER:-"https://uavcgbs1.selfhost.eu"}
ROOM_ID=${MATRIX_ROOM_ID:-"!deine_raum_id:uavcgbs1.selfhost.eu"}
TXID=$(date +%s)

JSON_BODY=$(cat <<EOF
{
  "msgtype": "m.text",
  "body": "$1"
}
EOF
)

curl -s -X PUT \
     -H "Authorization: Bearer $TOKEN" \
     -H "Content-Type: application/json" \
     --data-binary "$JSON_BODY" \
     "${HOMESERVER}_matrix/client/v3/rooms/$ROOM_ID/send/m.room.message/$TXID"