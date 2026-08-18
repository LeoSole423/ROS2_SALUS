import math
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np


def _is_number(value: Any) -> bool:
    try:
        number = float(value)
    except Exception:
        return False
    return math.isfinite(number)


def _coordinates_equal(a: List[float], b: List[float], tol: float = 1e-12) -> bool:
    return abs(a[0] - b[0]) <= tol and abs(a[1] - b[1]) <= tol


def _ring_area_abs(ring: List[List[float]]) -> float:
    if len(ring) < 4:
        return 0.0
    area = 0.0
    for idx in range(len(ring) - 1):
        x0, y0 = ring[idx]
        x1, y1 = ring[idx + 1]
        area += (x0 * y1) - (x1 * y0)
    return abs(area) * 0.5


def _normalize_ring(raw_ring: Any, context: str) -> List[List[float]]:
    if not isinstance(raw_ring, list):
        raise ValueError(f"{context}: ring must be a list")
    if len(raw_ring) < 3:
        raise ValueError(f"{context}: ring must have at least 3 positions")

    ring: List[List[float]] = []
    for idx, raw_position in enumerate(raw_ring):
        if not isinstance(raw_position, (list, tuple)) or len(raw_position) < 2:
            raise ValueError(f"{context}: invalid position at index {idx}")
        lon_raw = raw_position[0]
        lat_raw = raw_position[1]
        if not _is_number(lon_raw) or not _is_number(lat_raw):
            raise ValueError(f"{context}: non-numeric coordinates at index {idx}")
        lon = float(lon_raw)
        lat = float(lat_raw)
        if lon < -180.0 or lon > 180.0:
            raise ValueError(f"{context}: longitude out of range at index {idx}")
        if lat < -90.0 or lat > 90.0:
            raise ValueError(f"{context}: latitude out of range at index {idx}")
        ring.append([lon, lat])

    if len(ring) < 3:
        raise ValueError(f"{context}: ring has too few valid vertices")

    if not _coordinates_equal(ring[0], ring[-1]):
        ring.append([ring[0][0], ring[0][1]])

    if len(ring) < 4:
        raise ValueError(f"{context}: ring must have at least 4 positions after closure")

    if _ring_area_abs(ring) <= 1e-14:
        raise ValueError(f"{context}: degenerate ring with zero area")

    return ring


def _normalize_polygon(raw_polygon: Any, context: str) -> List[List[List[float]]]:
    if not isinstance(raw_polygon, list) or len(raw_polygon) == 0:
        raise ValueError(f"{context}: polygon coordinates must contain at least one ring")

    rings: List[List[List[float]]] = []
    for ring_idx, raw_ring in enumerate(raw_polygon):
        ring = _normalize_ring(raw_ring, f"{context}/ring[{ring_idx}]")
        rings.append(ring)
    return rings


def normalize_geojson_object(raw: Any) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        raise ValueError("GeoJSON root must be an object")

    raw_type = str(raw.get("type", ""))
    if raw_type == "FeatureCollection":
        raw_features = raw.get("features")
        if not isinstance(raw_features, list):
            raise ValueError("FeatureCollection.features must be a list")
        feature_candidates = raw_features
    elif raw_type == "Feature":
        feature_candidates = [raw]
    elif raw_type in ("Polygon", "MultiPolygon"):
        feature_candidates = [{"type": "Feature", "properties": {}, "geometry": raw}]
    else:
        raise ValueError(
            "GeoJSON root type must be FeatureCollection, Feature, Polygon or MultiPolygon"
        )

    normalized_features: List[Dict[str, Any]] = []
    for feature_idx, candidate in enumerate(feature_candidates):
        if not isinstance(candidate, dict) or str(candidate.get("type", "")) != "Feature":
            raise ValueError(f"Feature[{feature_idx}] is invalid")
        geometry = candidate.get("geometry")
        if not isinstance(geometry, dict):
            raise ValueError(f"Feature[{feature_idx}].geometry is missing or invalid")
        geometry_type = str(geometry.get("type", ""))
        coordinates = geometry.get("coordinates")

        if geometry_type == "Polygon":
            polygons = [_normalize_polygon(coordinates, f"Feature[{feature_idx}]/Polygon")]
        elif geometry_type == "MultiPolygon":
            if not isinstance(coordinates, list) or len(coordinates) == 0:
                raise ValueError(
                    f"Feature[{feature_idx}]/MultiPolygon requires a non-empty polygon list"
                )
            polygons = []
            for poly_idx, raw_polygon in enumerate(coordinates):
                polygons.append(
                    _normalize_polygon(
                        raw_polygon, f"Feature[{feature_idx}]/MultiPolygon[{poly_idx}]"
                    )
                )
        else:
            raise ValueError(
                f"Feature[{feature_idx}] geometry type '{geometry_type}' is not supported"
            )

        properties = candidate.get("properties")
        if properties is None:
            properties = {}
        if not isinstance(properties, dict):
            raise ValueError(f"Feature[{feature_idx}].properties must be an object")

        out_properties = dict(properties)
        out_properties["id"] = str(out_properties.get("id", f"zone_{feature_idx + 1}"))
        out_properties["type"] = str(out_properties.get("type", "no_go"))
        out_properties["enabled"] = bool(out_properties.get("enabled", True))

        if geometry_type == "Polygon":
            out_geometry = {"type": "Polygon", "coordinates": polygons[0]}
        else:
            out_geometry = {"type": "MultiPolygon", "coordinates": polygons}

        normalized_features.append(
            {
                "type": "Feature",
                "properties": out_properties,
                "geometry": out_geometry,
            }
        )

    return {"type": "FeatureCollection", "features": normalized_features}


def feature_and_polygon_counts(geojson_doc: Dict[str, Any]) -> Tuple[int, int]:
    features = geojson_doc.get("features", [])
    if not isinstance(features, list):
        return 0, 0
    polygon_count = 0
    for feature in features:
        if not isinstance(feature, dict):
            continue
        geometry = feature.get("geometry", {})
        geometry_type = str(geometry.get("type", ""))
        coords = geometry.get("coordinates", [])
        if geometry_type == "Polygon":
            polygon_count += 1
        elif geometry_type == "MultiPolygon" and isinstance(coords, list):
            polygon_count += len(coords)
    return len(features), polygon_count


def iter_polygons(geojson_doc: Dict[str, Any]) -> Iterable[Dict[str, Any]]:
    features = geojson_doc.get("features", [])
    if not isinstance(features, list):
        return

    for feature_idx, feature in enumerate(features):
        if not isinstance(feature, dict):
            continue
        geometry = feature.get("geometry", {})
        geometry_type = str(geometry.get("type", ""))
        coordinates = geometry.get("coordinates", [])
        properties = feature.get("properties", {})
        if not isinstance(properties, dict):
            properties = {}
        feature_id = str(properties.get("id", f"zone_{feature_idx + 1}"))
        zone_type = str(properties.get("type", "no_go"))
        enabled = bool(properties.get("enabled", True))

        if geometry_type == "Polygon":
            polygons = [coordinates]
        elif geometry_type == "MultiPolygon":
            polygons = coordinates if isinstance(coordinates, list) else []
        else:
            continue

        for polygon_idx, polygon in enumerate(polygons):
            if not isinstance(polygon, list) or len(polygon) == 0:
                continue
            outer = polygon[0]
            holes = polygon[1:]
            polygon_id = (
                feature_id
                if len(polygons) == 1
                else f"{feature_id}__{polygon_idx + 1}"
            )
            yield {
                "id": polygon_id,
                "feature_id": feature_id,
                "type": zone_type,
                "enabled": enabled,
                "outer_ll": outer,
                "holes_ll": holes,
            }


def _ring_xy_to_pixels(
    ring_xy: List[Dict[str, float]],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> Tuple[np.ndarray, int]:
    points: List[List[int]] = []
    clipped = 0
    for vertex in ring_xy:
        x = float(vertex["x"])
        y = float(vertex["y"])
        col = int(np.floor((x - origin_x) / resolution))
        row = int(np.floor((y - origin_y) / resolution))
        col_orig = col
        row_orig = row

        if col < 0:
            col = 0
        elif col >= width:
            col = width - 1
        if row < 0:
            row = 0
        elif row >= height:
            row = height - 1

        if col != col_orig or row != row_orig:
            clipped += 1

        img_row = height - 1 - row
        points.append([col, img_row])

    return np.array([points], dtype=np.int32), clipped


def rasterize_polygons_trinary(
    polygons_xy: List[Dict[str, Any]],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    buffer_margin_m: float = 0.0,
) -> Tuple[np.ndarray, Dict[str, int], List[str]]:
    # OpenCV se importa aca y no arriba porque el rasterizado es lo unico que lo
    # usa. Asi los helpers de GeoJSON de este modulo —pura geometria— se pueden
    # usar desde nodos que no tienen OpenCV instalado.
    import cv2

    image = np.full((height, width), 255, dtype=np.uint8)
    clipped_vertices: Dict[str, int] = {}
    outside_polygon_ids: List[str] = []

    if width <= 0 or height <= 0 or resolution <= 0.0:
        return image, clipped_vertices, outside_polygon_ids

    occupied = np.zeros((height, width), dtype=np.uint8)
    min_x = float(origin_x)
    min_y = float(origin_y)
    max_x = min_x + (float(width) * float(resolution))
    max_y = min_y + (float(height) * float(resolution))

    for polygon in polygons_xy:
        if polygon.get("enabled", True) is False:
            continue

        zone_id = str(polygon.get("id", "")) or "unnamed_zone"
        outer_xy = polygon.get("outer_xy", [])
        holes_xy = polygon.get("holes_xy", [])

        if len(outer_xy) < 3:
            continue

        xs = [float(v["x"]) for v in outer_xy]
        ys = [float(v["y"]) for v in outer_xy]
        for hole_ring in holes_xy:
            xs.extend(float(v["x"]) for v in hole_ring)
            ys.extend(float(v["y"]) for v in hole_ring)

        if max(xs) < min_x or min(xs) > max_x or max(ys) < min_y or min(ys) > max_y:
            outside_polygon_ids.append(zone_id)
            continue

        poly_mask = np.zeros_like(occupied)
        clipped_total = 0
        outer_pts, clipped = _ring_xy_to_pixels(
            outer_xy, width, height, resolution, origin_x, origin_y
        )
        clipped_total += clipped
        cv2.fillPoly(poly_mask, outer_pts, 255)

        for hole_xy in holes_xy:
            if len(hole_xy) < 3:
                continue
            hole_pts, clipped = _ring_xy_to_pixels(
                hole_xy, width, height, resolution, origin_x, origin_y
            )
            clipped_total += clipped
            cv2.fillPoly(poly_mask, hole_pts, 0)

        if clipped_total > 0:
            clipped_vertices[zone_id] = clipped_vertices.get(zone_id, 0) + clipped_total

        occupied = np.maximum(occupied, poly_mask)

    if buffer_margin_m > 0.0 and resolution > 0.0:
        radius_px = int(np.ceil(float(buffer_margin_m) / float(resolution)))
        if radius_px > 0:
            kernel_size = (2 * radius_px) + 1
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
            )
            occupied = cv2.dilate(occupied, kernel)

    image[occupied > 0] = 0
    return image, clipped_vertices, outside_polygon_ids
