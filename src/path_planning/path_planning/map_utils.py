#!/usr/bin/env python3

# ============================================================
# map_utils.py — carga de occupancy grids (formato map_server)
# y conversión entre coordenadas de píxel y metros.
#
# Convención del formato de mapas ROS (map_server):
#   - El .yaml da: image, resolution (m/px), origin [x, y, yaw],
#     negate, occupied_thresh, free_thresh.
#   - El origen del mapa (origin) es la esquina INFERIOR-izquierda
#     de la imagen en coordenadas del mundo.
#   - La fila 0 de la imagen es el borde SUPERIOR → hay que voltear
#     el eje Y al convertir píxel ↔ mundo.
# ============================================================

import os
import numpy as np
import yaml
from PIL import Image


class MapData:
    """Occupancy grid cargado + metadatos, con helpers px ↔ metros."""

    def __init__(self, img, resolution, origin):
        self.img = img                  # np.uint8 (H, W), 0=ocupado, 254=libre, 205=desconocido
        self.resolution = resolution    # m / px
        self.origin = origin            # [x, y, yaw] esquina inferior-izquierda
        self.height, self.width = img.shape

    def free_mask(self, free_value_min=250):
        """Máscara booleana del espacio libre (True = pista)."""
        return self.img >= free_value_min

    def px_to_world(self, rows, cols):
        """(fila, columna) de imagen → (x, y) en metros (frame map)."""
        rows = np.asarray(rows, dtype=np.float64)
        cols = np.asarray(cols, dtype=np.float64)
        x = self.origin[0] + (cols + 0.5) * self.resolution
        # fila 0 = borde superior → invertir eje Y
        y = self.origin[1] + (self.height - rows - 0.5) * self.resolution
        return x, y

    def world_to_px(self, x, y):
        """(x, y) en metros → (fila, columna) de imagen (float, sin redondear)."""
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        cols = (x - self.origin[0]) / self.resolution - 0.5
        rows = self.height - (y - self.origin[1]) / self.resolution - 0.5
        return rows, cols


def load_map(yaml_path):
    """Carga un mapa estilo map_server desde su .yaml (+imagen referida)."""
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)

    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)

    img = np.array(Image.open(img_path).convert('L'), dtype=np.uint8)
    if meta.get('negate', 0):
        img = 255 - img

    return MapData(img, float(meta['resolution']), list(meta['origin']))
