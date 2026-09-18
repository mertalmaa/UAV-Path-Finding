"""Ortam kontrolu: Python surumu, bagimliliklar, bolge verisi ve onbellek.

Baslaticinin "Sistem kontrolu" menusunden calisir, ama tek basina da calisir:

    python scripts/launcher/system_check.py

Cikis kodu: 0 = her sey tamam, 1 = en az bir zorunlu eksik.
"""
from __future__ import annotations

import importlib
import json
import platform
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]

MIN_PYTHON = (3, 9)

# (import adi, pip adi, zorunlu mu, ne icin)
PACKAGES = [
    ("numpy", "numpy", True, "planlayici cekirdegi"),
    ("scipy", "scipy", True, "arazi rehberi (Dijkstra)"),
    ("rasterio", "rasterio", True, "DEM okuma"),
    ("pyproj", "pyproj", True, "koordinat donusumu"),
    ("affine", "affine", True, "raster geometrisi"),
    ("PIL", "pillow", True, "Mission UI arazi karolari"),
    ("matplotlib", "matplotlib", False, "scripts/ grafikleri"),
    ("pytest", "pytest", False, "testler"),
]

REGION_FILES = ["working_dem.tif", "region_info.json"]

OK   = "  [ OK  ]"
WARN = "  [UYARI]"
FAIL = "  [EKSIK]"


def _version(mod) -> str:
    for attr in ("__version__", "version", "VERSION"):
        v = getattr(mod, attr, None)
        if isinstance(v, str):
            return v
    return "?"


def _mb(path: Path) -> float:
    return path.stat().st_size / 1_000_000


def check_python() -> bool:
    print("Python")
    print(f"  {sys.version.splitlines()[0]}")
    print(f"  {platform.platform()}")
    print(f"  {sys.executable}")
    if sys.version_info < MIN_PYTHON:
        print(f"{FAIL} en az Python {MIN_PYTHON[0]}.{MIN_PYTHON[1]} gerekli")
        return False
    in_venv = sys.prefix != getattr(sys, "base_prefix", sys.prefix)
    print(f"{OK} surum uygun" + ("  (sanal ortam icinde)" if in_venv else "  (sanal ortam yok)"))
    return True


def check_packages() -> bool:
    print("\nBagimliliklar")
    ok = True
    missing_required = []
    for import_name, pip_name, required, purpose in PACKAGES:
        try:
            mod = importlib.import_module(import_name)
        except Exception as exc:  # noqa: BLE001 - kurulum kontrolu
            tag = FAIL if required else WARN
            print(f"{tag} {pip_name:<12} yok ({purpose}) -- {type(exc).__name__}")
            if required:
                ok = False
                missing_required.append(pip_name)
            continue
        print(f"{OK} {pip_name:<12} {_version(mod):<10} {purpose}")
    if missing_required:
        print("\n  Kurulum:  pip install -r requirements.txt")
    return ok


def check_regions() -> bool:
    print("\nBolge verisi (regions/)")
    manifest_path = PROJECT_ROOT / "regions" / "regions_manifest.json"
    if not manifest_path.exists():
        print(f"{FAIL} regions/regions_manifest.json bulunamadi")
        return False
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    print(f"{OK} manifest: {len(manifest)} bolge ({', '.join(manifest)})")

    ok = False
    for region_id in manifest:
        region_dir = PROJECT_ROOT / "regions" / region_id
        missing = [f for f in REGION_FILES if not (region_dir / f).exists()]
        if missing:
            print(f"{WARN} {region_id:<10} eksik: {', '.join(missing)}")
            continue
        size = _mb(region_dir / "working_dem.tif")
        cache = region_dir / "terrain_cache" / "arrays.npz"
        note = f"onbellek {_mb(cache):.0f} MB" if cache.exists() else "onbellek yok (ilk calistirmada uretilir)"
        print(f"{OK} {region_id:<10} working_dem {size:.0f} MB, {note}")
        ok = True
    if not ok:
        print(f"{FAIL} kullanilabilir tek bir bolge bile yok")
    return ok


def check_optional() -> None:
    print("\nIstege bagli")
    cop = PROJECT_ROOT / "copernicus_glo30_turkey"
    if cop.is_dir():
        n = len(list(cop.glob("*.tif")))
        print(f"{OK} copernicus_glo30_turkey/ ({n} karo) -- yeni bolge uretilebilir")
    else:
        print(f"{WARN} copernicus_glo30_turkey/ yok -- Mission UI haritasinda golgelendirme "
              f"duz gorunur, planlama etkilenmez")

    tiles = PROJECT_ROOT / "mission_ui" / ".cache" / "terrain"
    if tiles.is_dir():
        n = sum(1 for _ in tiles.rglob("*.png"))
        print(f"{OK} arazi karo onbellegi: {n} karo")
    else:
        print(f"{WARN} arazi karo onbellegi yok -- karolar ilk bakista uretilir")


def main() -> int:
    print("=" * 62)
    print(" UAV Pathfinder -- sistem kontrolu")
    print(f" Proje koku: {PROJECT_ROOT}")
    print("=" * 62)
    results = [check_python(), check_packages(), check_regions()]
    check_optional()
    print("\n" + "=" * 62)
    if all(results):
        print(" Sonuc: ortam hazir.")
        return 0
    print(" Sonuc: eksikler var (yukaridaki [EKSIK] satirlarina bakin).")
    return 1


if __name__ == "__main__":
    sys.exit(main())
