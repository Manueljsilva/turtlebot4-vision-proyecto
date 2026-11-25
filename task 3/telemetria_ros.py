"""Salida de diagnóstico compartida; nunca esperar espacio en el pipe ROS."""

import os
import stat
import sys


def emitir_telemetria(texto):
    """Emite JSON sin bloquear el lazo de control si stdout apunta a un pipe lleno."""
    salida = sys.stdout
    if salida is None:
        return
    try:
        fd = salida.fileno()
    except (AttributeError, OSError, ValueError):
        fd = None
    if type(fd) is int and fd >= 0:
        try:
            modo = os.fstat(fd).st_mode
        except OSError:
            modo = 0
        if stat.S_ISFIFO(modo) or stat.S_ISSOCK(modo):
            datos = (texto + '\n').encode('utf-8')
            try:
                limite = os.fpathconf(fd, 'PC_PIPE_BUF') if hasattr(os, 'fpathconf') else 4096
            except (OSError, ValueError):
                limite = 4096
            if len(datos) <= limite:
                try:
                    os.set_blocking(fd, False)
                    os.write(fd, datos)
                except (BlockingIOError, BrokenPipeError, InterruptedError, OSError):
                    pass
            return
    try:
        salida.write(texto + '\n')
        salida.flush()
    except (BlockingIOError, BrokenPipeError, OSError, ValueError):
        pass

