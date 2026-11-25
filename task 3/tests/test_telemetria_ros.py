"""Canales locales POSIX: el diagnóstico no debe frenar el ciclo de control."""

import io
import json
import os
from pathlib import Path
import subprocess
import sys
import textwrap
import unittest
from unittest.mock import patch

from telemetria_ros import emitir_telemetria


class TelemetriaTests(unittest.TestCase):
    def test_captura_en_memoria_conserva_json_y_delimitador(self):
        with patch('sys.stdout',new_callable=io.StringIO) as salida:
            emitir_telemetria(json.dumps(dict(estado='detenido')))
        self.assertEqual(json.loads(salida.getvalue()),dict(estado='detenido'))
        self.assertTrue(salida.getvalue().endswith('\n'))

    @unittest.skipUnless(os.name == 'posix','Canales POSIX del Docker de Ubuntu')
    def test_pipe_lleno_no_bloquea_y_descartar_no_corrompe_el_siguiente_json(self):
        codigo=textwrap.dedent('''
            import io,json,os,sys,time
            sys.path.insert(0,sys.argv[1])
            from telemetria_ros import emitir_telemetria
            lector,escritor=os.pipe()
            os.set_blocking(escritor,False)
            while True:
                try:os.write(escritor,b'x'*4096)
                except BlockingIOError:break
            os.set_blocking(escritor,True)
            original=sys.stdout
            salida=os.fdopen(escritor,'w',closefd=False)
            sys.stdout=salida
            inicio=time.monotonic()
            emitir_telemetria(json.dumps({'estado':'pipe lleno'}))
            transcurrido=time.monotonic()-inicio
            os.set_blocking(lector,False)
            while True:
                try:os.read(lector,65536)
                except BlockingIOError:break
            emitir_telemetria(json.dumps({'grande':'x'*os.fpathconf(escritor,'PC_PIPE_BUF')}))
            emitir_telemetria(json.dumps({'estado':'recuperado'}))
            sys.stdout=original
            salida.close()
            os.close(escritor)
            datos=os.read(lector,65536)
            os.close(lector)
            print(json.dumps({'tiempo':transcurrido,'lineas':datos.decode().splitlines()}))
        ''')
        resultado=subprocess.run([sys.executable,'-B','-c',codigo,str(Path(__file__).resolve().parents[1])],
            text=True,capture_output=True,timeout=3,check=True)
        d=json.loads(resultado.stdout)
        self.assertLess(d['tiempo'],.5)
        self.assertEqual([json.loads(l) for l in d['lineas']],[dict(estado='recuperado')])

    @unittest.skipUnless(os.name == 'posix','Canales POSIX del Docker de Ubuntu')
    def test_pipe_sin_lector_no_interrumpe_control(self):
        lector,escritor=os.pipe()
        os.close(lector)
        try:
            with os.fdopen(escritor,'w',closefd=False) as salida, patch('sys.stdout',salida):
                emitir_telemetria('{"estado":"detenido"}')
        finally:
            os.close(escritor)
