#!/usr/bin/env bash
# Starts a virtual screen (:1) and serves it to your web browser on port 6080.

rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

Xvfb :1 -screen 0 1280x800x24 +extension GLX > /tmp/xvfb.log 2>&1 &
sleep 2

openbox > /tmp/openbox.log 2>&1 &

# The VNC server only accepts connections from inside the container.
x11vnc -display :1 -forever -shared -nopw -localhost -rfbport 5900 -quiet > /tmp/x11vnc.log 2>&1 &
sleep 1

websockify --web /usr/share/novnc 6080 localhost:5900 > /tmp/novnc.log 2>&1 &

echo "Desktop ready: open http://localhost:6080/vnc.html in your web browser."
exec sleep infinity