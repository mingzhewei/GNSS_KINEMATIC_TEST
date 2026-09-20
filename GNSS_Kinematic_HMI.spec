# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['gnss_kinematic_hmi.py'],
    pathex=[],
    binaries=[],
    datas=[('gps_kinematic_analyzer_beiyun.py', '.'), ('gps_kinematic_analyzer_huace.py', '.'), ('kinematic_core', 'kinematic_core')],
    hiddenimports=['matplotlib'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='GNSS_Kinematic_HMI',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='GNSS_Kinematic_HMI',
)
