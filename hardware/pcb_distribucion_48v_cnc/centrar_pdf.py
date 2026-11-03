#!/usr/bin/env python3
"""
centrar_pdf.py — Centra la placa en la hoja de montaje_1a1.pdf, sin cambiar la escala.

kicad-cli deja la placa pegada a la esquina de arriba a la izquierda del A4, y la
impresora corta ese borde. Esto la mueve al centro de la hoja.

    python3 centrar_pdf.py        # reescribe montaje_1a1.pdf
"""
import os

from pypdf import PdfReader, PdfWriter, Transformation

AQUI = os.path.dirname(os.path.abspath(__file__))
PDF = os.path.join(AQUI, 'montaje_1a1.pdf')
ANCHO_MM, ALTO_MM = 104.0, 84.0          # tiene que coincidir con gen.py
PT_POR_MM = 72 / 25.4


def main():
    pagina = PdfReader(PDF).pages[0]
    hoja_w, hoja_h = float(pagina.mediabox.width), float(pagina.mediabox.height)
    placa_w, placa_h = ANCHO_MM * PT_POR_MM, ALTO_MM * PT_POR_MM
    # En el PDF el eje Y crece hacia arriba: la placa parte en la esquina de arriba.
    dx = (hoja_w - placa_w) / 2
    dy = -(hoja_h - placa_h) / 2
    pagina.add_transformation(Transformation().translate(dx, dy))
    salida = PdfWriter()
    salida.add_page(pagina)
    with open(PDF, 'wb') as f:
        salida.write(f)
    print('centrado', PDF)


if __name__ == '__main__':
    main()
