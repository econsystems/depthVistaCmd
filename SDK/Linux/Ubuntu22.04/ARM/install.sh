#!/bin/bash
set -e

PREFIX="${1:-/}"
STARTDIR="$(pwd)"

LIBDIR="$PREFIX/usr/lib"
SHAREDIR="$PREFIX/usr/local/share"
INCLUDEDIR="$PREFIX/usr/local/include"

echo "Installing DepthVistaSDK"

mkdir -p "$LIBDIR"
mkdir -p "$INCLUDEDIR/DepthVistaSDK"

rm -f "$LIBDIR"/libDepthVistaSDK.*

cp -r "$STARTDIR/include/"* "$INCLUDEDIR/DepthVistaSDK/"
echo "Include done"

cp "$STARTDIR/so/"* "$LIBDIR/"
echo "Lib done"

cp "$STARTDIR/99-depthvista-permissions.rules" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

echo "Installing DepthVistaSDK success."
