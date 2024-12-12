#!/bin/bash

g++ -o modelCheckerUSD -g \
    src/components/Polygon.cpp \
    src/components/Vertex.cpp \
    src/components/UV.cpp \
    src/components/Edge.cpp \
    src/checks/getChecks.cpp \
    src/checks/commands.cpp \
    src/checks/overlappingMeshes.cpp \
    src/Formatter.cpp \
    src/main.cpp \
    src/Mesh.cpp \
    -std=c++17 \
    -I$HOME/libs \
    -I/usr/local/USD/include \
    -I/usr/local/include/ \
    -L/usr/local/USD/lib \
    -Wno-deprecated \
    -lusd_usd \
    -lusd_usdGeom \
    -lusd_sdf \
    -lusd_tf \
    -lusd_vt \
    -lusd_gf \
    -lboost_system \
    -ldl \
    -lusd_usdShade \
    -lgmp \
    -lmpfr

