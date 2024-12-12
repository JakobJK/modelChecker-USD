#!/bin/bash

echo "[" > compile_commands.json
for file in src/*.cpp; do
  echo "  {" >> compile_commands.json
  echo "    \"directory\": \"$(pwd)\"," >> compile_commands.json
  echo "    \"command\": \"g++ -o test $file -std=c++17 -I/usr/local/USD/include -L/usr/local/USD/lib -lusd_usd -lusd_usdGeom -lusd_sdf -lusd_tf -lusd_vt -Wno-deprecated -I/home/jake/libs/\"," >> compile_commands.json
  echo "    \"file\": \"$file\"" >> compile_commands.json
  echo "  }," >> compile_commands.json
done
sed -i '$ s/,$//' compile_commands.json # remove the last comma
echo "]" >> compile_commands.json

